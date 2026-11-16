"""Geometry-exact closed-loop IK and quasi-static load compensation. SIM ONLY.

The source MJCF supplies every link, hinge axis, contact and inertial value.
No Zuko/Kangal dimensions are copied. This verifies virtual actuator mapping,
not the actual physical servo channel/neutral/polarity. No robot I/O.
"""
from __future__ import annotations
import copy
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import mujoco
from scipy.optimize import least_squares, lsq_linear

LEGS = ('FL', 'FR', 'BL', 'BR')


def load_model(path, visuals=False):
    path = Path(path)
    root = ET.fromstring(path.read_text(encoding='utf-8-sig'))
    if root.find('compiler').get('inertiafromgeom') != 'false':
        raise ValueError('This controller requires explicit source inertias.')
    assets = {}
    if not visuals:
        for parent in root.iter():
            for geom in list(parent.findall('geom')):
                if geom.get('type') == 'mesh':
                    if geom.get('contype') != '0' or geom.get('conaffinity') != '0':
                        raise ValueError('Cannot omit a collision mesh.')
                    parent.remove(geom)
        for asset in root.findall('asset'):
            for mesh in list(asset.findall('mesh')):
                asset.remove(mesh)
    else:
        for mesh in root.findall('asset/mesh'):
            name = mesh.get('file')
            candidates = [path.parent/'cad_meshes'/name,
                          Path(__file__).resolve().parent/'output/cad_meshes'/name]
            found = next((p for p in candidates if p.is_file()), None)
            if found is None:
                raise FileNotFoundError('Missing visual mesh: '+str(name))
            assets[name] = found.read_bytes()
    xml = ET.tostring(root, encoding='unicode')
    return mujoco.MjModel.from_xml_string(xml, assets=assets), root


def smooth(value):
    value = float(np.clip(value, 0., 1.))
    return value*value*(3.-2.*value)


class LinkageRig:
    def __init__(self, source):
        self.source = Path(source).resolve()
        self.model, root = load_model(source)
        self.data = mujoco.MjData(self.model)
        # A separate planning state, never copied into the live simulation.
        self.planner, _ = load_model(source)
        self.planner.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
        self.kin = mujoco.MjData(self.planner)
        m = self.model
        free = m.joint('base_free').id
        if (m.nq, m.nv, m.nu) != (35, 34, 12) or m.jnt_qposadr[free] != 0:
            raise ValueError('Expected original free-root 35/34/12 model; do not use fixed swing XML.')
        if not np.allclose(m.opt.gravity, [0.,0.,-9.81]):
            raise ValueError('Gravity must be enabled in the source model.')
        if m.opt.disableflags & int(mujoco.mjtDisableBit.mjDSBL_CONTACT):
            raise ValueError('Source ground contacts must be enabled.')
        self.base = int(m.jnt_bodyid[free])
        self.floor = m.geom('visual_floor').id
        self.torso = m.geom('torso_collision').id
        self.joint_ids = m.actuator_trnid[:,0].astype(int)
        if len(set(self.joint_ids)) != 12 or not np.allclose(m.actuator_gear[:,0],1.):
            raise ValueError('Only twelve distinct direct joint position actuators are supported.')
        if not np.all(m.jnt_type[self.joint_ids] == mujoco.mjtJoint.mjJNT_HINGE):
            raise ValueError('All motor transmissions must address hinges.')
        self.qadr = m.jnt_qposadr[self.joint_ids].astype(int)
        self.dadr = m.jnt_dofadr[self.joint_ids].astype(int)
        self.passive = np.array([i for i in range(6,m.nv) if i not in self.dadr])
        self.independent = np.r_[np.arange(6), self.dadr]
        self.kp = m.actuator_gainprm[:,0].copy()
        if np.any(self.kp <= 0) or not np.allclose(m.actuator_biasprm[:,1],-self.kp):
            raise ValueError('Unsupported actuator law; expected position servo kp.')
        self.low, self.high = m.actuator_ctrlrange.T.copy()
        self.force_limit = np.maximum(abs(m.actuator_forcerange[:,0]),abs(m.actuator_forcerange[:,1]))
        if np.any(self.low >= 0) or np.any(self.high <= 0) or np.any(self.force_limit <= 0):
            raise ValueError('Invalid source motor ranges.')
        self.control_limit = np.minimum(-self.low,self.high)
        self.mapping = {}
        foot_geoms = [i for i in range(m.ngeom) if (m.geom(i).name or '').startswith('foot_collision_')]
        if len(foot_geoms) != 4:
            raise ValueError('Exactly four spherical toe contacts required.')
        for leg in LEGS:
            channels = [i for i in range(m.nu) if m.actuator(i).name.startswith(leg+'_')]
            coxa = [i for i in channels if m.actuator(i).name.endswith('_coxa')]
            if len(channels)!=3 or len(coxa)!=1:
                raise ValueError('Invalid virtual channel names for '+leg)
            ancestor = int(m.jnt_bodyid[self.joint_ids[coxa[0]]])
            matches = []
            for geom in foot_geoms:
                b = int(m.geom_bodyid[geom])
                while b and b != ancestor:
                    b = int(m.body_parentid[b])
                if b == ancestor: matches.append(geom)
            if len(matches)!=1: raise ValueError('Ambiguous foot ancestry: '+leg)
            geom = matches[0]
            site = m.site(m.geom(geom).name.replace('foot_collision_','foot_',1)).id
            self.mapping[leg] = dict(channels=channels,geom=geom,site=site)
        self.feet = np.array([self.mapping[l]['site'] for l in LEGS])
        self.geoms = np.array([self.mapping[l]['geom'] for l in LEGS])
        if not np.all(m.geom_type[self.geoms] == mujoco.mjtGeom.mjGEOM_SPHERE):
            raise ValueError('Toe clearance calculation requires sphere colliders.')
        self.pairs = [(m.site(e.get('site1')).id,m.site(e.get('site2')).id)
                      for e in root.findall('equality/connect')]
        if len(self.pairs)!=16: raise ValueError('Expected sixteen connect constraints.')
        self.sites = sorted(set(self.feet.tolist()+[i for pair in self.pairs for i in pair]))
        self.jac = {s: np.zeros((3,m.nv)) for s in self.sites}
        self.dt = .02
        self.substeps = round(self.dt/m.opt.timestep)
        if not np.isclose(self.substeps*m.opt.timestep,self.dt):
            raise ValueError('Physics timestep must divide 20ms.')
        self.weight = float(m.body_mass.sum()*9.81)

    def fk(self,q):
        self.kin.qpos[:] = q
        self.kin.qvel[:] = 0.
        mujoco.mj_kinematics(self.planner,self.kin)
        mujoco.mj_comPos(self.planner,self.kin)

    def jacobians(self):
        for site in self.sites:
            mujoco.mj_jacSite(self.planner,self.kin,self.jac[site],None,site)

    def inverse_kinematics(self,body_position,body_quat,foot_goals,warm):
        """Solve all 28 hinge coordinates + all 8 closed loops, not one fixed J."""
        q = warm.copy(); q[:3]=body_position; q[3:7]=body_quat
        cache = [None,None,None]
        def evaluate(x):
            if cache[0] is None or not np.array_equal(x,cache[0]):
                q[7:] = x; self.fk(q); self.jacobians()
                residual = np.r_[np.concatenate([10*(self.kin.site_xpos[a]-self.kin.site_xpos[b])
                    for a,b in self.pairs]), (self.kin.site_xpos[self.feet]-foot_goals).ravel()]
                J = np.vstack([10*(self.jac[a]-self.jac[b])[:,6:] for a,b in self.pairs]
                              +[self.jac[s][:,6:] for s in self.feet])
                cache[:] = [x.copy(),residual,J]
            return cache[1:]
        result = least_squares(lambda x:evaluate(x)[0],q[7:].copy(),
            jac=lambda x:evaluate(x)[1],max_nfev=16,ftol=1e-8,gtol=1e-10,xtol=1e-9)
        q[7:] = result.x; self.fk(q)
        error = float(np.linalg.norm(self.kin.site_xpos[self.feet]-foot_goals,axis=1).max())
        gap = max(float(np.linalg.norm(self.kin.site_xpos[a]-self.kin.site_xpos[b])) for a,b in self.pairs)
        if not np.all(np.isfinite(q)) or error>.001 or gap>.0001:
            raise RuntimeError(f'Closed-loop IK rejected: toe={error*1000:.3f}mm, loop={gap*1000:.3f}mm')
        return q,error

    def load_compensation(self,q,stance):
        """Virtual work: eliminate passive hinges, balance gravity with foot forces.

        Computes a position-setpoint offset tau/kp; never applies an external
        root force, changes actuator limits or writes actual joint positions.
        """
        self.fk(q); self.jacobians()
        C = np.vstack([self.jac[a]-self.jac[b] for a,b in self.pairs])
        N = np.zeros((self.model.nv,18)); N[self.independent,np.arange(18)] = 1.
        N[self.passive] = np.linalg.lstsq(C[:,self.passive],-C[:,self.independent],rcond=1e-8)[0]
        mujoco.mj_forward(self.planner,self.kin)
        gravity = N.T@self.kin.qfrc_bias
        J = np.vstack([self.jac[self.feet[i]] for i in stance])@N
        balance = J[2::3,:6].T
        normals = lsq_linear(balance,gravity[:6],bounds=(0,np.inf),tol=1e-8,lsq_solver='exact').x
        torque = gravity[6:]-J[2::3,6:].T@normals
        residual = float(np.linalg.norm(balance@normals-gravity[:6]))
        return q[self.qadr]+torque/self.kp,torque,normals,residual

    def snapshot(self):
        m,d=self.model,self.data
        mujoco.mj_forward(m,d)
        normal = np.zeros(4); torso=False
        for i in range(d.ncon):
            c=d.contact[i]
            if self.floor not in (c.geom1,c.geom2): continue
            other = c.geom2 if c.geom1==self.floor else c.geom1
            torso |= other==self.torso
            for k,g in enumerate(self.geoms):
                if other==g:
                    f=np.zeros(6);mujoco.mj_contactForce(m,d,i,f)
                    world=c.frame.reshape(3,3).T@f[:3]
                    normal[k]+=max(0.,float(world[2])*(1 if c.geom2==g else -1))
        R=d.xmat[self.base].reshape(3,3)
        return dict(position=d.xpos[self.base].copy(),feet=d.site_xpos[self.feet].copy(),
                    forces=normal,clearance=d.site_xpos[self.feet,2]-m.geom_size[self.geoms,0],
                    com=d.subtree_com[self.base].copy(),tilt=float(np.degrees(np.arccos(np.clip(R[2,2],-1,1)))),
                    torso_contact=bool(torso),loop_gap=max(float(np.linalg.norm(d.site_xpos[a]-d.site_xpos[b])) for a,b in self.pairs))

    def settle(self):
        for _ in range(round(2.5/self.model.opt.timestep)):
            mujoco.mj_step(self.model,self.data)
        state=self.snapshot()
        if state['position'][2]<.075 or state['tilt']>12 or sum(state['forces']>2)<3:
            raise RuntimeError('Source does not stand under zero control. No crawl starts.')
        if state['loop_gap']>.001 or np.any(self.data.warning.number):
            raise RuntimeError('Invalid source dynamics during settle.')
        return state
