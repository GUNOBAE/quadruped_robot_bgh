"""All-direction, velocity-aware receding-horizon crawl. SIMULATION ONLY.
This is controller-parameter optimization, not PPO. No live pose rewriting,
external base forces, changed masses, motor strengths or contact softness.
"""
from __future__ import annotations
import os
for _k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS'):os.environ[_k]='1'
import itertools,json,math,sys,time,traceback
from pathlib import Path
import numpy as np
import mujoco
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation
from linkage_core import LinkageRig,LEGS
from omni_trial import OmniCrawl,DIRECTIONS,DEFAULT as OMNI,rz,yaw_of,write_json
from flow_trial import FlowCrawl,hermite5

COMMANDS=tuple(d for d in DIRECTIONS if d!='stop')
ORDERS=tuple((0,)+p for p in itertools.permutations((1,2,3)))
DEFAULT=dict(stride_mm=20.,lift_mm=18.,slot_s=1.1,swing_fraction=.67,drift=.30,
             support_fraction=.10,path_weight=.30,height_mm=112.,velocity_ff=.65,
             anchor_blend=.25,order=4,yaw_deg=1.5)
BOUNDS=dict(stride_mm=(6.,36.),lift_mm=(14.,21.),slot_s=(.65,1.85),swing_fraction=(.60,.78),
            drift=(.12,.50),support_fraction=(.075,.12),path_weight=(.10,.60),
            height_mm=(106.,118.),velocity_ff=(0.,1.),anchor_blend=(0.,.50),order=(0,5),yaw_deg=(.5,3.))

def valid(p):
    if set(p)!=set(DEFAULT):raise ValueError('Unexpected control parameter keys')
    for k,(lo,hi) in BOUNDS.items():
        if not math.isfinite(float(p[k])) or not lo<=p[k]<=hi:raise ValueError('Invalid '+k)
    if int(p['order'])!=p['order']:raise ValueError('order must be an integer')

def progress_metrics(w,start,initial,direction,cycles,stride_mm,yaw_deg):
    rows=w.rows[start:];dt=w.r.dt
    if len(rows)<10:raise RuntimeError('Insufficient telemetry')
    f,l,turn=DIRECTIONS[direction]
    heading=rz(initial[3]).as_matrix()@np.array([l,-f,0.]);normal=np.array([-heading[1],heading[0]])
    xyz=np.array([r['pos'] for r in rows]);xy=xyz[:,:2]-np.asarray(initial[:2]);yaw=np.unwrap([r['yaw'] for r in rows])
    if turn:
        displacement=(yaw-initial[3])*turn;target=math.radians(yaw_deg)*cycles
    else:
        displacement=xy@heading[:2];target=stride_mm/1000*cycles
    lag=5;vel=(displacement[lag:]-displacement[:-lag])/(dt*lag)
    stationary=abs(vel)<(math.radians(.05) if turn else .001)
    streak=longest=0
    for value in stationary:
        streak=streak+1 if value else 0;longest=max(longest,streak)
    vv=np.diff(xyz[:,:2],axis=0)/dt;acc=np.diff(vv,axis=0)/dt;jerk=np.diff(acc,axis=0)/dt
    work=sum(r['power'] for r in rows)*dt;dist=float(displacement[-1]);duration=len(rows)*dt
    trace=xy@normal if not turn else np.linalg.norm(xy,axis=1)
    units=180/math.pi if turn else 1000.
    return dict(speed=units*dist/duration,speed_unit='deg/s' if turn else 'mm/s',
        distance=units*dist,distance_unit='deg' if turn else 'mm',target=target*units,ratio=dist/target,
        duration_s=duration,forward_fraction=float(np.mean(vel>(math.radians(.05) if turn else .001))),
        reverse_fraction=float(np.mean(vel<-(math.radians(.05) if turn else .001))),
        stall_fraction=float(np.mean(stationary)),longest_stall_s=longest*dt,
        sway_mm=float(np.ptp(trace)*1000),cross_drift_mm=float(abs(trace[-1])*1000),
        yaw_error_deg=float(abs(math.atan2(math.sin(yaw[-1]-initial[3]),math.cos(yaw[-1]-initial[3])))*180/math.pi) if not turn else 0.,
        accel_rms=float(np.sqrt(np.mean(np.sum(acc*acc,axis=1)))),
        jerk_rms=float(np.sqrt(np.mean(np.sum(jerk*jerk,axis=1)))),
        max_tilt_deg=max(r['tilt'] for r in rows),tilt_rms=float(np.sqrt(np.mean([r['tilt']**2 for r in rows]))),
        work_J=work,work_per_progress=work/max(dist,1e-9),
        clip_fraction=float(np.mean([r['clip'] for r in rows])),
        torque_sat_fraction=float(np.mean([r['force_sat'] for r in rows])),
        loop_gap_mm=max(r['loop_gap'] for r in rows)*1000,
        warning='Mechanical work sampled at 50Hz; not battery consumption. Playback speed is fixed at 1x.')

class StrideCrawl(OmniCrawl):
    def __init__(self,r,p):
        valid(p);self.cfg=dict(p)
        super().__init__(r,dict(OMNI,stride_mm=min(18.,p['stride_mm']),lift_mm=min(18.,p['lift_mm']),curve=5))
        self.height=p['height_mm']/1000;self.qd_filtered=np.zeros(12);self.prev_boundary_velocity=np.zeros(3)
        self.cycle_progress=[];self.reanchors=[];self.measurements=[]

    def support_pose(self,leg,landing):
        r=self.r;p=self.cfg;goals=self.anchors.copy();goals[leg]=(goals[leg]+landing)/2;goals[leg,2]+=p['lift_mm']/1000
        stance=[i for i in range(4) if i!=leg];center=self.center();warm=self.qref.copy();values=[]
        quat=rz(self.yaw).as_quat()[[3,0,1,2]];expected=self.expected_body.copy();expected[2]=self.height
        def cost(pos):
            if np.linalg.norm(pos[:2]-center[:2])>.065 or not .095<pos[2]<.13:return 100.
            try:q,_=r.inverse_kinematics(pos,quat,goals,warm)
            except RuntimeError:return 100.
            ctrl,tau,normals,balance=r.load_compensation(q,stance)
            use=max(float(np.max(abs(ctrl)/(.96*r.control_limit))),float(np.max(abs(tau)/(.94*r.force_limit))))
            delta=(pos-expected)[:2]/.025
            score=.35*use+20*max(0.,p['support_fraction']*r.weight-min(normals))/r.weight+balance
            score+=p['path_weight']*float(delta@delta)+10*max(0.,use-.95)**2+((pos[2]-self.height)/.03)**2
            values.append((float(score),pos.copy(),use,float(min(normals))))
            return float(score)
        R=rz(self.yaw).as_matrix()
        for x in (-.02,0.,.02):
            for y in (-.02,0.,.02):cost(center+R@np.array([x,y,0.]))
        if not values:raise RuntimeError('No reachable support pose')
        minimize(cost,min(values,key=lambda x:x[0])[1],method='Nelder-Mead',
                 options={'maxiter':90,'xatol':.0001,'fatol':.0001})
        best=min(values,key=lambda x:x[0])
        if best[2]>1.18 or best[3]<.035*r.weight:raise RuntimeError('Support plan exceeds guarded load/angle margin')
        self.plans.append(dict(leg=LEGS[leg],position=best[1].tolist(),use=best[2],force_N=best[3]))
        return best[1]

    def control(self,body,goals,stance,phase,leg=-1,blend=1.):
        r=self.r;m,d=r.model,r.data;R=d.xmat[r.base].reshape(3,3).copy();wanted=rz(self.yaw)
        error=np.r_[body-d.xpos[r.base],Rotation.from_matrix(wanted.as_matrix()@R.T).as_rotvec()]
        self.integral+=r.dt*error;self.integral[:3]=np.clip(self.integral[:3],-.008,.008);self.integral[3:]=np.clip(self.integral[3:],-.05,.05)
        vb=body+.45*error[:3]+.6*self.integral[:3];rotation=Rotation.from_rotvec(.45*error[3:]+.6*self.integral[3:])*wanted
        targets=goals.copy()
        for i in range(4):
            if i not in stance:targets[i]=vb+rotation.as_matrix()@R.T@(goals[i]-d.xpos[r.base])
        last=self.qref.copy();self.qref,ik=r.inverse_kinematics(vb,rotation.as_quat()[[3,0,1,2]],targets,self.qref)
        request,tau,normals,balance=r.load_compensation(self.qref,stance)
        if blend<1.:
            four,_,_,_=r.load_compensation(self.qref,[0,1,2,3]);request=four*(1-blend)+request*blend
        # Compensate known simulator servo damping for desired motion, NOT a
        # larger motor/gain. Source control/force bounds and rate limit stay on.
        velocity=(self.qref[r.qadr]-last[r.qadr])/r.dt
        self.qd_filtered=.6*self.qd_filtered+.4*velocity
        request+=self.cfg['velocity_ff']*(-m.actuator_biasprm[:,2])*np.clip(self.qd_filtered,-2.,2.)/r.kp
        d.ctrl[:]=np.clip(d.ctrl+np.clip(request-d.ctrl,-.02,.02),r.low,r.high)
        warn=d.warning.number.copy();old_t=d.time
        for _ in range(r.substeps):mujoco.mj_step(m,d)
        s=r.snapshot();self.t+=r.dt
        if (np.any(d.warning.number>warn) or d.time<=old_t or
            not np.all(np.isfinite(d.qpos)) or not np.all(np.isfinite(d.qvel))):raise RuntimeError('Numerical warning/reset')
        self.rows.append(dict(time=self.t,phase=phase,command=self.command,leg=leg,pos=s['position'].tolist(),
            feet=s['feet'].tolist(),forces=s['forces'].tolist(),clearance=s['clearance'].tolist(),
            tilt=s['tilt'],yaw=yaw_of(d.xmat[r.base].reshape(3,3)),loop_gap=s['loop_gap'],
            ctrl=d.ctrl.tolist(),q=d.qpos[r.qadr].tolist(),tau=d.actuator_force.tolist(),
            power=float(np.sum(abs(d.actuator_force*d.qvel[r.dadr]))),
            clip=float(np.mean((request<r.low)|(request>r.high))),force_sat=float(np.mean(abs(d.actuator_force)>=.99*r.force_limit))))
        self.states.append(d.qpos.copy())
        if s['tilt']>25 or s['position'][2]<.075 or s['torso_contact'] or s['loop_gap']>.0015:
            raise RuntimeError('Unsafe simulated support/contact state')

    def motion(self,direction,cycles):
        if direction=='stop':
            self.command=direction;before=self.r.data.xpos[self.r.base].copy();begin=len(self.rows)
            ya=yaw_of(self.r.data.xmat[self.r.base].reshape(3,3))
            for _ in range(100):self.control(self.base,self.anchors,[0,1,2,3],'STOP')
            drift=np.linalg.norm(self.r.data.xpos[self.r.base][:2]-before[:2])
            yb=yaw_of(self.r.data.xmat[self.r.base].reshape(3,3))
            if drift>.005 or max(r['tilt'] for r in self.rows[begin:])>10 or abs(math.atan2(math.sin(yb-ya),math.cos(yb-ya)))>.02:
                raise RuntimeError('Stop stability gate failed')
            return None
        p=self.cfg;self.command=direction;stride=p['stride_mm']/1000;slot=p['slot_s'];duty=p['swing_fraction']
        f,l,turn=DIRECTIONS[direction];dyaw=math.radians(p['yaw_deg'])*turn
        if not self.prepared:self.shift(self.center(),2.,'READY');self.prepared=True
        start=len(self.rows);xyz0=self.r.data.xpos[self.r.base].copy();yaw0=yaw_of(self.r.data.xmat[self.r.base].reshape(3,3))
        initial=np.r_[xyz0,yaw0];order=ORDERS[int(p['order'])];lo=(1-duty)/2;hi=1-lo
        reference_origin=self.base.copy();initial_yaw=self.yaw;last_xyz=xyz0;last_yaw=yaw0
        incoming=np.zeros(3)
        for cycle in range(cycles):
            anchors=self.anchors.copy();q0=self.qref.copy();b0=self.base.copy();current_yaw=self.yaw
            heading=rz(current_yaw).as_matrix()@np.array([l,-f,0.]);translation=heading*stride
            center=np.r_[anchors[:,:2].mean(axis=0),0.]
            landings=(rz(dyaw).as_matrix()@(anchors-center).T).T+center+translation
            poses=[]
            # Replan from current contact anchors every full cycle. The live
            # state is never assigned the planner solution.
            for j,leg in enumerate(order):
                self.yaw=current_yaw+dyaw*(j+.5)/4
                self.expected_body=reference_origin+heading*stride*(cycle+(j+.5)/4)
                pose=self.support_pose(leg,landings[leg]);poses.append(pose)
                self.base=pose.copy();self.anchors[leg]=landings[leg]
                self.qref,_=self.r.inverse_kinematics(pose,rz(self.yaw).as_quat()[[3,0,1,2]],self.anchors,self.qref)
            self.anchors=anchors;self.base=b0;self.qref=q0;self.yaw=current_yaw
            velocity=heading*stride*p['drift']/slot
            ts=[0.];points=[b0];vels=[incoming]
            for j,pose in enumerate(poses):
                ts.extend([(j+lo)*slot,(j+hi)*slot]);points.extend([pose-velocity*duty*slot/2,pose+velocity*duty*slot/2]);vels.extend([velocity,velocity])
            outgoing=np.zeros(3) if cycle==cycles-1 else velocity
            ts.append(4*slot);points.append(poses[-1]+velocity*slot*.5);vels.append(outgoing)
            ts=np.asarray(ts);points=np.asarray(points);vels=np.asarray(vels)
            def path(t):
                j=min(len(ts)-2,max(0,int(np.searchsorted(ts,t,side='right'))-1));T=ts[j+1]-ts[j]
                return hermite5(points[j],points[j+1],vels[j],vels[j+1],T,(t-ts[j])/T)
            for j,leg in enumerate(order):
                before=len(self.rows);target=landings[leg];old=self.anchors[leg].copy();stance=[i for i in range(4) if i!=leg]
                n=round(slot/self.r.dt)
                for k in range(n):
                    ph=(k+1)/n;body=path((j+ph)*slot);goals=self.anchors.copy()
                    self.yaw=current_yaw+dyaw*(j+ph)/4
                    if lo<=ph<=hi:
                        u=(ph-lo)/duty;goals[leg]=old+(target-old)*self.easing(u)
                        # C2 lift/hold/lower. Longer useful clearance without
                        # lowering the fixed >=0.2s and >=8mm actual air gate.
                        z=self.easing(u/.35) if u<.35 else self.easing((1-u)/.35) if u>.65 else 1.
                        goals[leg,2]+=p['lift_mm']/1000*z;phase='ARC_SWING';support=stance;blend=1.
                    elif ph<lo:
                        blend=self.easing((ph-lo+.12)/.12);support=stance if blend>0 else [0,1,2,3];phase='TRANSFER'
                    else:
                        goals[leg]=target;blend=1-self.easing((ph-hi)/.12);support=stance if blend>0 else [0,1,2,3];phase='LAND'
                    self.control(body,goals,support,phase,leg,blend)
                rows=self.rows[before:];air=[r for r in rows if r['clearance'][leg]>.008 and r['forces'][leg]<1.]
                force=float(np.mean([r['forces'][leg] for r in rows[-6:]]));supported=min((sum(r['forces'][i]>2 for i in stance) for r in air),default=0)
                actual=np.asarray(rows[-1]['feet'][leg]);err=float(np.linalg.norm(actual-target));speed=float(np.linalg.norm(actual-np.asarray(rows[-6]['feet'][leg]))/.10)
                check=dict(leg=LEGS[leg],air_s=len(air)*self.r.dt,contact_N=force,support=int(supported),placement_mm=1000*err,
                    passed=bool(len(air)*self.r.dt>=.2 and force>2 and supported==3 and err<.008))
                self.steps.append(check);print('STEP',cycle+1,LEGS[leg],check,flush=True)
                if not check['passed']:raise RuntimeError('Real foot lift/support/placement failed')
                self.anchors[leg]=target;self.base=body.copy()
                # Correct only a confirmed, settled touchdown, at most 1mm XY.
                # Do NOT erase placement error or count the correction as travel.
                correction=np.zeros(3)
                if speed<.01:
                    correction[:2]=np.clip((actual-target)[:2]*p['anchor_blend'],-.001,.001)
                    self.anchors[leg]+=correction
                self.reanchors.append(dict(leg=LEGS[leg],correction_mm=(correction*1000).tolist(),foot_speed_mps=speed))
            self.yaw=current_yaw+dyaw;incoming=outgoing
            xyz=self.r.data.xpos[self.r.base].copy();ya=yaw_of(self.r.data.xmat[self.r.base].reshape(3,3))
            ratio=float((xyz-last_xyz)@heading/stride) if not turn else math.atan2(math.sin(ya-last_yaw),math.cos(ya-last_yaw))/dyaw
            self.cycle_progress.append(ratio);last_xyz=xyz;last_yaw=ya
            if 0<cycle<cycles-1 and ratio<.4:raise RuntimeError('Incomplete phase-matched cycle progress')
        self.shift(self.center(),1.,'FINISH',yaw=initial_yaw+cycles*dyaw)
        m=progress_metrics(self,start,initial,direction,cycles,p['stride_mm'],p['yaw_deg']);self.measurements.append(m)
        return m


def execute(task):
    out=Path(task['out']);out.mkdir(parents=True,exist_ok=True);w=None;error=None;metrics=None;start_wall=time.monotonic()
    snapshots=[];segs=[]
    try:
        if not 1<=int(task['cycles'])<=32:raise ValueError('cycles must be 1..32')
        rig=LinkageRig(task['source']);mu=float(task.get('friction',1.))
        if mu not in (.9,1.,1.1):raise ValueError('Unknown holdout')
        rig.model.geom_friction[np.r_[rig.geoms,rig.floor],0]*=mu
        kind=task.get('kind','new');direction=task['direction'];cycles=int(task['cycles'])
        if kind=='new':
            w=StrideCrawl(rig,task['params'])
            for d in task.get('commands',[direction]):
                if task.get('command_params') and d!='stop':w.cfg=dict(task['command_params'][d]);w.height=w.cfg['height_mm']/1000
                metrics=w.motion(d,cycles);segs.append(dict(direction=d,metrics=metrics))
        elif kind=='flow':
            w=FlowCrawl(rig,task['params']);s,xyz,stride=w.run_flow(direction,cycles)
            metrics=progress_metrics(w,s,np.r_[xyz,0.],direction,cycles,stride*1000,0.)
        elif kind=='omni':
            w=OmniCrawl(rig,task['params']);w.shift(w.center(),2.,'READY');w.prepared=True
            s=len(w.rows);xyz=rig.data.xpos[rig.base].copy();ya=yaw_of(rig.data.xmat[rig.base].reshape(3,3))
            w.segment(direction,cycles);metrics=progress_metrics(w,s,np.r_[xyz,ya],direction,cycles,task['params']['stride_mm'],task['params']['yaw_deg'])
        else:raise ValueError('Unknown controller kind')
        ms=[x['metrics'] for x in segs if x['metrics']] if segs else [metrics]
        for m in ms:
            if not (.6<=m['ratio']<=1.4 and m['cross_drift_mm']<10 and m['max_tilt_deg']<10 and
                    m['clip_fraction']<.12 and m['torque_sat_fraction']<.05 and m['yaw_error_deg']<3):
                raise RuntimeError('Direction/posture/actuator validation failed')
    except Exception:error=traceback.format_exc();print(error,flush=True)
    if w and w.rows:
        np.savez_compressed(out/'trajectory.npz',qpos=np.asarray(w.states),dt=w.r.dt)
        np.savez_compressed(out/'telemetry.npz',time=[r['time'] for r in w.rows],pos=[r['pos'] for r in w.rows],
             feet=[r['feet'] for r in w.rows],forces=[r['forces'] for r in w.rows],clearance=[r['clearance'] for r in w.rows],
             ctrl=[r['ctrl'] for r in w.rows],tau=[r['tau'] for r in w.rows])
    r=dict(passed=error is None,direction=task['direction'],cycles=task['cycles'],friction=task.get('friction',1.),
           params=task['params'],kind=task.get('kind','new'),metrics=metrics,segments=segs,failure=error,
           steps=w.steps if w else [],cycle_progress=getattr(w,'cycle_progress',[]),wall_s=time.monotonic()-start_wall,simulation_only=True,PPO=False,
           source_physics_unchanged=True,reanchors=getattr(w,'reanchors',[]))
    write_json(out/'report.json',r);print('RESULT',r['passed'],r['direction'],metrics,flush=True);return r

if __name__=='__main__':execute(json.loads(Path(sys.argv[1]).read_text(encoding='utf-8-sig')))
