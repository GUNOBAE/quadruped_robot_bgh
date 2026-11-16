"""Budgeted locomotion lab. SIMULATION ONLY; NO HARDWARE I/O.

Loads existing Physics V1/RL XML without changing its dynamics, gains, torque
or control limits. Training can omit visual-only meshes because every body
already has explicit inertial properties. Old PPO files are NOT compatible
with this 59-observation experiment, and are never modified.
"""
from __future__ import annotations
import copy
import hashlib
import math
from pathlib import Path
from xml.etree import ElementTree as ET
import gymnasium as gym
from gymnasium import spaces
import mujoco
import numpy as np

LEGS = ('FL', 'BR', 'FR', 'BL')
CONFIGS = {
    'A_reference': {'reward': 'legacy', 'residual': False, 'bounded_std': False},
    'B_low_noise': {'reward': 'legacy', 'residual': False, 'bounded_std': True},
    'C_posture': {'reward': 'progress', 'residual': False, 'bounded_std': True},
    'D_gait_residual': {'reward': 'progress', 'residual': True, 'bounded_std': True},
}
COMMANDS = {'stop': (0., 0., 0.), 'forward': (.04, 0., 0.),
            'backward': (-.04, 0., 0.), 'left': (0., .03, 0.),
            'right': (0., -.03, 0.), 'turn_left': (0., 0., .3),
            'turn_right': (0., 0., -.3)}


def load_xml(path: str | Path, visuals: bool = False):
    path = Path(path)
    text = path.read_text(encoding='utf-8-sig')
    root = ET.fromstring(text)
    if not visuals:
        if root.find('compiler').get('inertiafromgeom') != 'false':
            raise ValueError('Explicit inertias required before omitting visuals.')
        for parent in root.iter():
            for geom in list(parent.findall('geom')):
                if geom.get('type') == 'mesh':
                    if geom.get('contype') != '0' or geom.get('conaffinity') != '0':
                        raise ValueError('Cannot omit a mesh used for collision.')
                    parent.remove(geom)
        asset = root.find('asset')
        if asset is not None:
            for mesh in list(asset.findall('mesh')):
                asset.remove(mesh)
    assets = {}
    if visuals:
        for mesh in root.findall('asset/mesh'):
            name = mesh.get('file')
            candidates = [path.parent / 'cad_meshes' / name,
                          Path(__file__).resolve().parent / 'output/cad_meshes' / name]
            found = next((p for p in candidates if p.is_file()), None)
            if found is None:
                raise FileNotFoundError('Missing CAD mesh: ' + str(name))
            assets[name] = found.read_bytes()
    xml = ET.tostring(root, encoding='unicode')
    return mujoco.MjModel.from_xml_string(xml, assets=assets), root


def name_id(model, kind, name):
    result = mujoco.mj_name2id(model, kind, name)
    if result < 0:
        raise ValueError('Missing model element: ' + name)
    return result


def identify(model):
    j = name_id(model, mujoco.mjtObj.mjOBJ_JOINT, 'base_free')
    if int(model.jnt_qposadr[j]) != 0 or model.nu != 12:
        raise ValueError('Expected free base at qpos[0] and 12 actuators.')
    base = int(model.jnt_bodyid[j])
    floor = name_id(model, mujoco.mjtObj.mjOBJ_GEOM, 'visual_floor')
    feet = [i for i in range(model.ngeom)
            if (model.geom(i).name or '').startswith('foot_collision_')]
    if len(feet) != 4:
        raise ValueError('Exactly four toe spheres required.')
    mapping = {}
    for leg in LEGS:
        channels = [i for i in range(model.nu)
                    if model.actuator(i).name.startswith(leg + '_')]
        if len(channels) != 3:
            raise ValueError('Three virtual channels required for ' + leg)
        coxa = next(i for i in channels if model.actuator(i).name.endswith('_coxa'))
        cbody = int(model.jnt_bodyid[int(model.actuator_trnid[coxa, 0])])
        descendants = []
        for geom in feet:
            body = int(model.geom_bodyid[geom])
            while body and body != cbody:
                body = int(model.body_parentid[body])
            if body == cbody:
                descendants.append(geom)
        if len(descendants) != 1:
            raise ValueError('Ambiguous toe ancestry for ' + leg)
        geom = descendants[0]
        site = name_id(model, mujoco.mjtObj.mjOBJ_SITE,
                       model.geom(geom).name.replace('foot_collision_', 'foot_', 1))
        mapping[leg] = {'channels': channels, 'geom': geom, 'site': site}
    return base, floor, mapping


def calibrate_gait(source, amplitude=.016, lift=.020, period=2.4):
    """Local setpoint-to-foot Jacobians; never applied to actual hardware."""
    _, root = load_xml(source)
    fixed = copy.deepcopy(root)
    for body in fixed.findall('.//body'):
        free = body.find('freejoint')
        if free is not None:
            body.remove(free)
    option = fixed.find('option')
    option.set('gravity', '0 0 0')
    flag = option.find('flag')
    if flag is None:
        flag = ET.SubElement(option, 'flag')
    flag.set('contact', 'disable')
    model = mujoco.MjModel.from_xml_string(ET.tostring(fixed, encoding='unicode'))
    _, _, mapping = identify(load_xml(source)[0])
    out = {'amplitude': amplitude, 'lift': lift, 'period': period, 'legs': {}}
    def probe(channel, angle, site):
        data = mujoco.MjData(model)
        data.ctrl[channel] = angle
        for _ in range(round(.45 / model.opt.timestep)):
            mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)
        return data.site_xpos[site].copy()
    for leg, part in mapping.items():
        # Names survive free-joint removal; resolve site ids on the fixed copy.
        site_name = load_xml(source)[0].site(part['site']).name
        site = name_id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        J = np.column_stack([(probe(i, .06, site)-probe(i, -.06, site))/.12
                             for i in part['channels']])
        condition = float(np.linalg.cond(J))
        if not np.isfinite(condition) or condition > 100:
            raise ValueError('Ill-conditioned foot Jacobian: ' + leg)
        out['legs'][leg] = {'channels': part['channels'], 'inverse': np.linalg.pinv(J).tolist(),
                            'condition': condition, 'jacobian': J.tolist()}
    return out


class DayEnv(gym.Env):
    metadata = {'render_modes': []}

    def __init__(self, source, variant='C_posture', gait=None, command=None,
                 randomize=False, visuals=False, episode_seconds=6., all_directions=False):
        super().__init__()
        self.source = Path(source)
        self.model, root = load_xml(source, visuals)
        self.base, self.floor, self.mapping = identify(self.model)
        if not np.allclose(self.model.opt.gravity, [0, 0, -9.81]):
            raise ValueError('DayLab requires gravity ON. Never pass fixed-base swing XML.')
        if self.model.opt.disableflags & int(mujoco.mjtDisableBit.mjDSBL_CONTACT):
            raise ValueError('DayLab requires contacts ON.')
        self.data = mujoco.MjData(self.model)
        self.variant = variant
        self.config = CONFIGS[variant]
        self.gait = gait
        if self.config['residual'] and gait is None:
            raise ValueError('Residual policy requires calibrated gait.')
        self.fixed_command, self.randomize = command, randomize
        self.all_directions = all_directions
        self.dt = .02
        self.skip = round(self.dt / self.model.opt.timestep)
        if not np.isclose(self.skip * self.model.opt.timestep, self.dt):
            raise ValueError('Physics timestep must divide 20 ms.')
        self.max_steps = round(episode_seconds/self.dt)
        self.channels = self.model.actuator_trnid[:, 0].astype(int)
        self.qadr = self.model.jnt_qposadr[self.channels].astype(int)
        self.dadr = self.model.jnt_dofadr[self.channels].astype(int)
        self.geom_ids = np.array([self.mapping[l]['geom'] for l in LEGS])
        self.site_ids = np.array([self.mapping[l]['site'] for l in LEGS])
        self.pairs = [(name_id(self.model, mujoco.mjtObj.mjOBJ_SITE, e.get('site1')),
                       name_id(self.model, mujoco.mjtObj.mjOBJ_SITE, e.get('site2')))
                      for e in root.findall('equality/connect')]
        if len(self.pairs) != 16:
            raise ValueError('Expected sixteen loop constraints.')
        self.torso = name_id(self.model, mujoco.mjtObj.mjOBJ_GEOM, 'torso_collision')
        self.default_mu = self.model.geom_friction.copy()
        self.default_solref = self.model.geom_solref.copy()
        self.limit = np.minimum(.22, np.minimum(-self.model.actuator_ctrlrange[:, 0],
                                               self.model.actuator_ctrlrange[:, 1]))
        if np.min(self.limit) <= 0:
            raise ValueError('Expected symmetric zero-centred position commands.')
        for _ in range(round(1.0/self.model.opt.timestep)):
            mujoco.mj_step(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self.rest_q = self.data.qpos.copy()
        self.rest_height = float(self.data.xpos[self.base, 2])
        self.rest_upright = float(self.data.xmat[self.base].reshape(3,3)[2,2])
        self.action_space = spaces.Box(-1., 1., shape=(12,), dtype=np.float32)
        self.observation_space = spaces.Box(-10., 10., shape=(59,), dtype=np.float32)
        self.count = 0
        self.ctrl = np.zeros(12)
        self.prev_action = np.zeros(12)
        self.command = np.zeros(3)
        self.velocity = np.zeros(3)
        self.yaw_rate = 0.
        self.contact = np.zeros(4)
        self.clearance = np.zeros(4)
        self.last_info = {}

    def rotation(self):
        return self.data.xmat[self.base].reshape(3,3)

    def contacts(self):
        flags = np.zeros(4)
        torso = False
        lookup = {int(g): k for k,g in enumerate(self.geom_ids)}
        for c in self.data.contact[:self.data.ncon]:
            a,b = int(c.geom1),int(c.geom2)
            if self.floor not in (a,b):
                continue
            other = b if a == self.floor else a
            torso |= other == self.torso
            if other in lookup:
                flags[lookup[other]] = 1.
        return flags, torso

    def observe(self):
        R = self.rotation()
        # qvel angular entries of the root are local angular velocities.
        gyro = self.data.qvel[3:6]
        phase = 2*math.pi*self.count*self.dt/(self.gait['period'] if self.gait else 2.4)
        h = float(self.data.xpos[self.base,2])
        obs = np.concatenate((R.T @ [0.,0.,-1.], gyro/8., self.velocity/.5,
            self.data.qpos[self.qadr]/.25, self.data.qvel[self.dadr]/8.,
            self.ctrl/.25, self.command/[.1,.1,.8], [math.sin(phase),math.cos(phase)],
            self.contact, [(h-self.rest_height)/.05], self.clearance/.04)).astype(np.float32)
        if obs.shape != (59,) or not np.all(np.isfinite(obs)):
            raise RuntimeError('Invalid DayLab observation.')
        return np.clip(obs,-10,10)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model,self.data)
        self.model.geom_friction[:] = self.default_mu
        self.model.geom_solref[:] = self.default_solref
        if self.randomize:
            mu = float(self.np_random.uniform(.8,1.6))
            self.model.geom_friction[self.geom_ids,0] = mu
            self.model.geom_friction[self.floor,0] = mu
            tc = float(self.np_random.uniform(.018,.04))
            self.model.geom_solref[self.geom_ids,0] = tc
            self.model.geom_solref[self.floor,0] = tc
        self.data.qpos[:] = self.rest_q
        self.data.qpos[:2] += self.np_random.uniform(-.002,.002,2)
        self.data.qvel[:] = 0.
        self.data.ctrl[:] = 0.
        mujoco.mj_forward(self.model,self.data)
        if self.fixed_command is not None:
            self.command = np.array(COMMANDS[self.fixed_command],float)
        else:
            choices = list(COMMANDS) if self.all_directions else ['stop','forward','backward','forward','backward']
            self.command = np.array(COMMANDS[str(self.np_random.choice(choices))],float)
        self.count = 0
        self.ctrl[:] = 0.
        self.prev_action[:] = 0.
        self.velocity[:] = 0.
        self.yaw_rate = 0.
        self.contact,_ = self.contacts()
        self.clearance = np.maximum(0.,self.data.site_xpos[self.site_ids,2]-self.model.geom_size[self.geom_ids,0])
        return self.observe(), {'simulation_only': True}

    def reference(self):
        targets = np.zeros(12)
        if self.gait is None or np.linalg.norm(self.command) < 1e-9:
            return targets
        t = self.count*self.dt
        scale = min(1.,t/.5)
        period = self.gait['period']
        # Sequential crawl-shaped reference, NOT a validated gait/controller.
        for k,leg in enumerate(LEGS):
            phase = (t/period-k/4.) % 1.
            if phase < .25:
                u = phase/.25
                offset = -.5 + .5*(1-math.cos(math.pi*u))
                lift = self.gait['lift']*math.sin(math.pi*u)**2
            else:
                u = (phase-.25)/.75
                offset = .5-u
                lift = 0.
            vx,vy,w = self.command
            base_xyz = self.model.body_pos[self.model.geom_bodyid[self.mapping[leg]['geom']]]
            direction = np.array([vy-w*base_xyz[1], -vx+w*base_xyz[0]])
            norm = np.linalg.norm(direction)
            direction = direction/max(.04,norm)
            dp = np.array([direction[0]*self.gait['amplitude']*offset,
                           direction[1]*self.gait['amplitude']*offset,lift])*scale
            part = self.gait['legs'][leg]
            targets[part['channels']] = np.asarray(part['inverse']) @ dp
        return np.clip(targets,-self.limit,self.limit)

    def step(self, action):
        a = np.asarray(action,dtype=float)
        if a.shape != (12,) or not np.all(np.isfinite(a)):
            raise ValueError('Action must be finite vector[12].')
        a = np.clip(a,-1,1)
        reference = self.reference() if self.config['residual'] else np.zeros(12)
        requested = reference + (.06*a if self.config['residual'] else .22*a)
        requested = np.clip(requested,-self.limit,self.limit)
        self.ctrl += np.clip(requested-self.ctrl,-.03,.03)
        before = self.data.xpos[self.base].copy()
        oldR = self.rotation().copy()
        feet_before = self.data.site_xpos[self.site_ids].copy()
        old_t = float(self.data.time)
        warning_before = self.data.warning.number.copy()
        self.data.ctrl[:] = self.ctrl
        for _ in range(self.skip):
            mujoco.mj_step(self.model,self.data)
        # mj_step integrates qpos; refresh geometry before measuring new pose.
        mujoco.mj_forward(self.model,self.data)
        if (not np.all(np.isfinite(self.data.qpos)) or not np.all(np.isfinite(self.data.qvel))
            or self.data.time <= old_t or np.any(self.data.warning.number > warning_before)):
            raise RuntimeError('Numerical warning/reset: reject this experiment, not a fall.')
        self.count += 1
        R = self.rotation()
        world_v = (self.data.xpos[self.base]-before)/self.dt
        local = R.T @ world_v
        self.velocity = np.array([-local[1],local[0],local[2]])
        yaw0 = math.atan2(-oldR[0,1],oldR[1,1])
        yaw1 = math.atan2(-R[0,1],R[1,1])
        self.yaw_rate = math.atan2(math.sin(yaw1-yaw0),math.cos(yaw1-yaw0))/self.dt
        self.contact,torso = self.contacts()
        self.clearance = np.maximum(0.,self.data.site_xpos[self.site_ids,2]-self.model.geom_size[self.geom_ids,0])
        foot_v = (self.data.site_xpos[self.site_ids]-feet_before)/self.dt
        slip = float(np.mean(np.sum(foot_v[:,:2]**2,axis=1)*self.contact))
        height = float(self.data.xpos[self.base,2])
        upright = float(np.clip(R[2,2],-1,1))
        tilt = math.degrees(math.acos(upright))
        gap = max(np.linalg.norm(self.data.site_xpos[i]-self.data.site_xpos[j]) for i,j in self.pairs)
        if gap > .002:
            raise RuntimeError('Closed-loop constraint separation exceeds 2mm.')
        movement = self.velocity[:2]
        c = self.command[:2]
        tracking = math.exp(-min(50.,float(np.sum(((movement-c)/.045)**2))))
        zero_tracking = math.exp(-float(np.sum((c/.045)**2)))
        yaw_track = math.exp(-min(50.,((self.yaw_rate-self.command[2])/.3)**2))
        smooth = float(np.mean((a-self.prev_action)**2))
        work = float(np.mean(np.abs(self.data.actuator_force*self.data.qvel[self.dadr])))
        fall = bool(height < max(.06,.72*self.rest_height) or tilt>40 or torso)
        if self.config['reward']=='legacy':
            reward = 2*tracking+.7*yaw_track+.45*max(0,upright)+.2*np.clip((height-.045)/.065,0,1)-.015*smooth-.008*work-.025*slip
        else:
            signed = float(np.dot(movement,c)/max(float(np.dot(c,c)),.0016))
            # Stationary robot gets no free velocity credit during movement commands.
            reward = 2*(tracking-zero_tracking)+.5*np.clip(signed,-1,1.5)
            reward -= 2*np.clip(((height-self.rest_height)/.03)**2,0,4)
            reward -= 2*(1-upright)+.1*min(5.,((self.yaw_rate-self.command[2])/.3)**2)
            reward -= .02*smooth+.004*work+.25*min(slip,2.)+.03*float(np.mean(a*a))
            if np.linalg.norm(self.command)<1e-8:
                reward += .5*tracking
        if fall:
            reward -= 5.
        self.prev_action = a.copy()
        force_limit = np.maximum(np.abs(self.model.actuator_forcerange[:,0]),np.abs(self.model.actuator_forcerange[:,1]))
        self.last_info = {'height_m':height,'tilt_deg':tilt,'fall':fall,'torso_contact':bool(torso),
            'velocity':self.velocity.tolist(),'yaw_rate':self.yaw_rate,'contacts':self.contact.tolist(),
            'clearance_m':self.clearance.tolist(),'slip':slip,'loop_gap_m':float(gap),
            'action_sat':float(np.mean(np.abs(a)>=.9)),
            'target_sat':float(np.mean(np.abs(self.ctrl)>=.95*self.limit)),
            'force_sat':float(np.mean(np.abs(self.data.actuator_force)>=.95*force_limit)),
            'joint_error_deg':float(np.rad2deg(np.mean(np.abs(self.ctrl-self.data.qpos[self.qadr]))))}
        return self.observe(),float(reward),fall,self.count>=self.max_steps,self.last_info

    def close(self):
        pass


def evaluate(source, variant, gait, policy=None, commands=('forward','backward','stop'),
             seeds=(501,502), seconds=6., randomize=False):
    rows=[]
    for command in commands:
        env=DayEnv(source,variant,gait,command=command,randomize=randomize,episode_seconds=seconds)
        for seed in seeds:
            obs,_=env.reset(seed=int(seed))
            start=env.data.xpos[env.base].copy()
            R0=env.rotation().copy()
            trace=[]
            history=[]
            episode_reward=0.
            for _ in range(env.max_steps):
                a=np.zeros(12) if policy is None else policy.predict(obs,deterministic=True)[0]
                obs,r,term,trunc,info=env.step(a)
                episode_reward+=r
                trace.append(dict(info))
                local=R0.T @ (env.data.xpos[env.base]-start)
                history.append([env.count*env.dt,-local[1],local[0],float(info['height_m'])])
                if term or trunc:
                    break
            tail=trace[-min(len(trace),max(1,round(2/env.dt))):]
            avg=np.mean([x['velocity'] for x in tail],axis=0)
            cmd=np.asarray(COMMANDS[command])
            axis=0 if command in ('forward','backward','stop') else 1
            goal=float(cmd[axis])
            achieved=float(avg[axis])
            ratio=achieved/goal if goal else 0.
            lift_events=sum(sum(trace[i]['clearance_m'][k]>.008 and trace[i-1]['clearance_m'][k]<=.008
                                for k in range(4)) for i in range(1,len(trace)))
            down=env.rest_height-float(np.mean([x['height_m'] for x in tail]))
            max_tilt=max(x['tilt_deg'] for x in trace)
            full=len(trace)>=env.max_steps
            fell=bool(trace[-1]['fall'])
            cross=float(avg[1-axis])
            candidate=bool(command!='stop' and full and not fell and .5<=ratio<=1.5
                           and abs(cross)<.025 and down<.015 and max_tilt<20 and lift_events>=2)
            score=(float(np.clip(ratio,-1,1.2))-max(0,down)/.02-max_tilt/80
                   -.1*float(np.mean([x['target_sat'] for x in tail]))-float(fell)*2
                   -.25*float(lift_events<2)) if goal else -float(np.linalg.norm(avg[:2]))
            rows.append({'command':command,'seed':int(seed),'seconds':len(trace)*env.dt,
                'tail_velocity_mps':avg.tolist(),'target_velocity':cmd.tolist(),
                'tail_primary_ratio':ratio,'displacement_forward_left_m':history[-1][1:3],
                'height_drop_vs_zero_m':down,'max_tilt_deg':max_tilt,'fell':fell,
                'lift_events':int(lift_events),'action_sat':float(np.mean([x['action_sat'] for x in tail])),
                'target_sat':float(np.mean([x['target_sat'] for x in tail])),
                'force_sat':float(np.mean([x['force_sat'] for x in tail])),
                'reward':episode_reward,'candidate_not_certification':candidate,
                'score':float(score),'samples':history})
        env.close()
    moving=[r for r in rows if r['command'] in ('forward','backward')]
    return {'simulation_only':True,'variant':variant,'episodes':rows,
            'score':float(np.mean([r['score'] for r in moving])),
            'candidate_rate':float(np.mean([r['candidate_not_certification'] for r in moving])),
            'fall_rate':float(np.mean([r['fell'] for r in moving])),
            'tail_ratio':float(np.mean([r['tail_primary_ratio'] for r in moving]))}
