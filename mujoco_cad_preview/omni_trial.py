"""Multidirectional closed-linkage crawl experiments. SIMULATION ONLY, not PPO.
Source masses/gains/contact/limits stay unchanged. No live qpos or root forces.
"""
from __future__ import annotations
import os
for _name in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS'):
    os.environ[_name]='1'
import json, math, sys, time, traceback
from pathlib import Path
import numpy as np
import mujoco
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation
from linkage_core import LinkageRig, LEGS

DIRECTIONS = {
    'forward': (1.,0.,0.), 'backward': (-1.,0.,0.),
    'left': (0.,1.,0.), 'right': (0.,-1.,0.),
    'forward_left': (2**-.5,2**-.5,0.), 'forward_right': (2**-.5,-2**-.5,0.),
    'backward_left': (-2**-.5,2**-.5,0.), 'backward_right': (-2**-.5,-2**-.5,0.),
    'turn_left': (0.,0.,1.), 'turn_right': (0.,0.,-1.), 'stop': (0.,0.,0.),
}
DEFAULT = dict(stride_mm=12., lift_mm=15., shift_s=1., unload_s=.35,
               swing_scale=.6, land_s=.5, yaw_deg=1.5, travel_weight=0., curve=5)
BOUNDS = dict(stride_mm=(4.,18.), lift_mm=(12.,18.), shift_s=(.6,2.),
              unload_s=(.25,.5), swing_scale=(.4,1.), land_s=(.3,.7),
              yaw_deg=(.4,3.), travel_weight=(0.,.25),curve=(3,5))

def write_json(path, data):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_suffix(path.suffix+'.tmp')
    temp.write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    temp.replace(path)

def validate_params(p):
    if set(p)!=set(DEFAULT):raise ValueError('Unexpected parameter fields')
    for k,(lo,hi) in BOUNDS.items():
        if not math.isfinite(float(p[k])) or not lo<=p[k]<=hi:raise ValueError('Out of range '+k)
    if p['curve'] not in (3,5):raise ValueError('curve must be 3 or 5')

def rz(yaw):return Rotation.from_euler('z',yaw)
def yaw_of(R):return math.atan2(R[1,0],R[0,0])

class OmniCrawl:
    def __init__(self,rig,params):
        validate_params(params);self.r=rig;self.p=dict(params)
        s=rig.settle();self.qref=rig.data.qpos.copy();self.anchors=s['feet'].copy()
        self.base=s['position'].copy();self.com0=np.r_[s['com'][:2]-self.base[:2],0.]
        self.yaw=0.;self.height=.115;self.integral=np.zeros(6)
        self.rows=[];self.states=[];self.steps=[];self.plans=[];self.segments=[]
        self.t=0.;self.command='stop';self.prepared=False

    def easing(self,x):
        x=float(np.clip(x,0.,1.))
        return x*x*(3.-2.*x) if self.p['curve']==3 else x*x*x*(10.+x*(-15.+6.*x))

    def center(self):
        return np.r_[self.anchors[:,:2].mean(axis=0)-(rz(self.yaw).apply(self.com0))[:2],self.height]

    def support_pose(self,leg,landing):
        r=self.r;goals=self.anchors.copy();goals[leg]=(goals[leg]+landing)/2
        goals[leg,2]+=self.p['lift_mm']/1000
        stance=[i for i in range(4) if i!=leg];center=self.center();warm=self.qref.copy();values=[]
        quat=rz(self.yaw).as_quat()[[3,0,1,2]];rot=rz(self.yaw).as_matrix()
        def cost(pos):
            if np.linalg.norm(pos[:2]-center[:2])>.065 or not .09<pos[2]<.14:return 100.
            try:q,_=r.inverse_kinematics(pos,quat,goals,warm)
            except RuntimeError:return 100.
            ctrl,tau,normals,balance=r.load_compensation(q,stance)
            use=max(float(np.max(abs(ctrl)/(.96*r.control_limit))),float(np.max(abs(tau)/(.94*r.force_limit))))
            score=use+10*max(0.,.114*r.weight-min(normals))/r.weight+balance
            score+=.4*np.linalg.norm(pos[:2]-center[:2])
            # Normalize travel penalty, unlike the almost negligible old metre penalty.
            score+=self.p['travel_weight']*np.linalg.norm(pos[:2]-self.base[:2])/.04
            values.append((float(score),pos.copy(),use,float(min(normals))))
            return float(score)
        for x in (-.025,-.0125,0.,.0125,.025):
            for y in (-.02,0.,.02):
                for h in (.11,.115,.12):cost(center+rot@np.array([x,y,h-self.height]))
        if not values:raise RuntimeError('No reachable three-foot support pose')
        minimize(cost,min(values,key=lambda a:a[0])[1],method='Nelder-Mead',
                 options={'maxiter':70,'xatol':.0002,'fatol':.0005})
        best=min(values,key=lambda a:a[0])
        self.plans.append(dict(leg=LEGS[leg],position=best[1].tolist(),use=best[2],min_force_N=best[3]))
        if best[2]>1.18 or best[3]<.04*r.weight:raise RuntimeError('Support load outside baseline gate')
        return best[1]

    def control(self,body,goals,stance,phase,leg=-1,blend=1.):
        r=self.r;m,d=r.model,r.data;R=d.xmat[r.base].reshape(3,3).copy();wanted=rz(self.yaw)
        error=np.r_[body-d.xpos[r.base],Rotation.from_matrix(wanted.as_matrix()@R.T).as_rotvec()]
        self.integral+=r.dt*error;self.integral[:3]=np.clip(self.integral[:3],-.008,.008)
        self.integral[3:]=np.clip(self.integral[3:],-.05,.05)
        vb=body+.45*error[:3]+.6*self.integral[:3]
        rotation=Rotation.from_rotvec(.45*error[3:]+.6*self.integral[3:])*wanted
        quat=rotation.as_quat()[[3,0,1,2]];targets=goals.copy()
        for i in range(4):
            if i not in stance:targets[i]=vb+rotation.as_matrix()@R.T@(goals[i]-d.xpos[r.base])
        self.qref,ik_error=r.inverse_kinematics(vb,quat,targets,self.qref)
        request,tau,normals,balance=r.load_compensation(self.qref,stance)
        if blend<1.:
            four,_,_,_=r.load_compensation(self.qref,[0,1,2,3]);request=four*(1-blend)+request*blend
        # Never write d.qpos, d.qvel or applied root force in a physical trial.
        d.ctrl[:]=np.clip(d.ctrl+np.clip(request-d.ctrl,-.02,.02),r.low,r.high)
        warn=d.warning.number.copy();old_t=d.time
        for _ in range(r.substeps):mujoco.mj_step(m,d)
        s=r.snapshot();self.t+=r.dt
        if np.any(d.warning.number>warn) or d.time<=old_t or not np.all(np.isfinite(d.qpos)):
            raise RuntimeError('Numerical warning/reset')
        v=d.qvel[r.dadr];power=float(np.sum(abs(d.actuator_force*v)))
        current_R=d.xmat[r.base].reshape(3,3)
        self.rows.append(dict(time=self.t,phase=phase,command=self.command,leg=leg,
            pos=s['position'].tolist(),feet=s['feet'].tolist(),forces=s['forces'].tolist(),
            clearance=s['clearance'].tolist(),tilt=s['tilt'],yaw=yaw_of(current_R),
            loop_gap=s['loop_gap'],torso=bool(s['torso_contact']),ik_error=ik_error,
            ctrl=d.ctrl.tolist(),q=d.qpos[r.qadr].tolist(),tau=d.actuator_force.tolist(),
            power=power,clip=float(np.mean((request<r.low)|(request>r.high))),
            force_sat=float(np.mean(abs(d.actuator_force)>=.99*r.force_limit))))
        self.states.append(d.qpos.copy())
        if s['tilt']>25 or s['position'][2]<.075 or s['torso_contact'] or s['loop_gap']>.0015:
            raise RuntimeError('Unsafe simulated support state')

    def shift(self,target,duration,phase='SHIFT',yaw=None):
        old=self.base.copy();old_yaw=self.yaw;end_yaw=self.yaw if yaw is None else yaw
        n=max(1,round(duration/.02))
        for k in range(n):
            s=self.easing((k+1)/n);self.yaw=old_yaw+(end_yaw-old_yaw)*s
            self.control(old+(target-old)*s,self.anchors,[0,1,2,3],phase)
        self.base=target.copy();self.yaw=end_yaw

    def step_leg(self,leg,landing):
        p=self.p;initial=self.anchors.copy();start=len(self.rows);target=landing.copy()
        stance=[i for i in range(4) if i!=leg];up=.6*p['swing_scale'];travel=.8*p['swing_scale'];down=up
        unload=p['unload_s'];total=unload+up+travel+down;peak=initial[leg,2]+p['lift_mm']/1000
        n=max(1,round(total/.02))
        for k in range(n):
            t=(k+1)*total/n;goals=initial.copy();blend=1.
            if t<unload:blend=self.easing(t/unload)
            elif t<unload+up:goals[leg,2]+=(peak-initial[leg,2])*self.easing((t-unload)/up)
            elif t<unload+up+travel:
                goals[leg]=initial[leg]+(target-initial[leg])*self.easing((t-unload-up)/travel);goals[leg,2]=peak
            else:
                goals[leg]=target;goals[leg,2]=peak+(target[2]-peak)*self.easing((t-unload-up-travel)/down)
            self.control(self.base,goals,stance,'SWING',leg,blend)
        self.anchors[leg]=target
        for _ in range(max(1,round(p['land_s']/.02))):self.control(self.base,self.anchors,[0,1,2,3],'LAND',leg)
        rows=self.rows[start:];air=[x for x in rows if x['phase']=='SWING' and x['clearance'][leg]>.008 and x['forces'][leg]<1.]
        contact=float(np.mean([x['forces'][leg] for x in rows[-min(10,len(rows)):]]))
        support=min((sum(x['forces'][j]>2. for j in stance) for x in air),default=0)
        err=float(np.linalg.norm(np.asarray(rows[-1]['feet'][leg])-target))
        step=dict(leg=LEGS[leg],airborne_s=len(air)*.02,touchdown_N=contact,support_count=int(support),
                  error_mm=1000*err,peak_mm=1000*max(x['clearance'][leg] for x in rows),
                  passed=bool(len(air)*.02>=.2 and contact>2. and support==3 and err<.008))
        self.steps.append(step);print('STEP',json.dumps(step),flush=True)
        if not step['passed']:raise RuntimeError('Lift/support/placement gate failed')

    def segment(self,command,cycles):
        self.command=command;fwd,left,turn=DIRECTIONS[command]
        if not self.prepared:
            self.shift(self.center(),2.,'READY');self.prepared=True
        start=len(self.rows)-1;start_step=len(self.steps);p0=np.asarray(self.rows[-1]['pos']);yaw0=self.rows[-1]['yaw']
        origin_yaw=self.yaw;expected=np.zeros(2);expected_yaw=0.;waypoints=[(start,p0.copy())]
        if command=='stop':
            for _ in range(100):self.control(self.base,self.anchors,[0,1,2,3],'STOP')
        else:
            for cycle in range(cycles):
                R=rz(self.yaw).as_matrix();delta=R@np.array([left,-fwd,0.])*self.p['stride_mm']/1000
                dyaw=math.radians(self.p['yaw_deg'])*turn;old_yaw=self.yaw
                cen=np.r_[self.anchors[:,:2].mean(axis=0),0.]
                landings=(rz(dyaw).as_matrix()@(self.anchors-cen).T).T+cen+delta
                # Foot order fixed across directions for initial comparison.
                for k,leg in enumerate((0,3,1,2)):
                    # Target yaw follows the already replanted support geometry.
                    self.yaw=old_yaw+dyaw*k/4
                    target=self.support_pose(leg,landings[leg])
                    previous=old_yaw+dyaw*max(0,k-1)/4 if k else old_yaw
                    self.yaw=previous
                    self.shift(target,self.p['shift_s'],yaw=old_yaw+dyaw*k/4)
                    self.step_leg(leg,landings[leg])
                self.shift(self.center(),.4,'CENTER',yaw=old_yaw+dyaw)
                expected+=delta[:2];expected_yaw+=dyaw
                waypoints.append((len(self.rows)-1,np.r_[p0[:2]+expected,p0[2]]))
                print('CYCLE',cycle+1,command,flush=True)
            self.shift(self.center(),1.,'FINISH',yaw=origin_yaw+expected_yaw)
            waypoints[-1]=(len(self.rows)-1,waypoints[-1][1])
        rows=self.rows[start+1:];p1=np.asarray(rows[-1]['pos']);yaw1=rows[-1]['yaw']
        actual=p1[:2]-p0[:2];angle=math.atan2(math.sin(yaw1-yaw0),math.cos(yaw1-yaw0))
        length=float(np.linalg.norm(expected));duration=len(rows)*.02
        along=float(np.dot(actual,expected/length)) if length else 0.
        cross=abs(float(expected[0]*actual[1]-expected[1]*actual[0]))/length if length else float(np.linalg.norm(actual))
        ratio=along/length if length else angle/expected_yaw if expected_yaw else 1.
        tilt=max(x['tilt'] for x in rows);work=sum(x['power'] for x in rows)*.02
        # Track deviation from segment's initial-to-final line (not sensor noise).
        normal=np.array([-expected[1],expected[0]])/length if length else None
        xy=np.asarray([x['pos'][:2] for x in rows])-p0[:2]
        sway=float(np.ptp(xy@normal)) if normal is not None else float(np.max(np.linalg.norm(xy,axis=1)))
        rms_tilt=float(np.sqrt(np.mean([x['tilt']**2 for x in rows])))
        clip=float(np.mean([x['clip'] for x in rows]));force=float(np.mean([x['force_sat'] for x in rows]))
        progress=[]
        for (ia,pa),(ib,pb) in zip(waypoints,waypoints[1:]):
            a=np.array(self.rows[ib]['pos'])-np.array(self.rows[ia]['pos']);goal=pb-pa
            if np.linalg.norm(goal[:2])>1e-8:progress.append(float(np.dot(a[:2],goal[:2])/np.dot(goal[:2],goal[:2])))
        complete=len(self.steps)-start_step==4*cycles if command!='stop' else True
        okay=bool(complete and .6<=ratio<=1.4 and tilt<10 and cross<.01 and clip<.12 and force<.05)
        if not turn:okay &= abs(angle)<math.radians(3.)
        if progress:okay &= min(progress)>=.4
        if command=='stop':okay=bool(tilt<10 and np.linalg.norm(actual)<.005 and abs(angle)<.02)
        report=dict(command=command,cycles=cycles,passed=bool(okay),ratio=ratio,duration_s=duration,
            expected_xy_m=expected.tolist(),actual_xy_m=actual.tolist(),expected_yaw_rad=expected_yaw,actual_yaw_rad=angle,
            progress_each_cycle=progress,net_distance_mm=1000*along,cross_drift_mm=1000*cross,
            speed_mm_s=1000*along/duration if length else None,yaw_deg_s=math.degrees(angle)/duration,
            max_tilt_deg=tilt,rms_tilt_deg=rms_tilt,lateral_sway_pp_mm=1000*sway,
            mean_clip=clip,mean_force_sat=force,work_J=work,
            acceleration_rms_mps2=float(np.sqrt(np.mean(np.sum(np.diff(xy,n=2,axis=0)**2,axis=1)))/.02**2) if len(xy)>2 else 0.,
            mechanical_cot_proxy=work/(self.r.weight*along) if along>1e-6 else None,
            placement_max_mm=max((x['error_mm'] for x in self.steps[start_step:]),default=0.))
        self.segments.append(report)
        if not okay:raise RuntimeError('Segment direction/posture/progress gate failed')
        return report


def execute(task):
    out=Path(task['out']);out.mkdir(parents=True,exist_ok=True);walker=None;failure=None;t0=time.monotonic()
    try:
        rig=LinkageRig(task['source']);factor=float(task.get('friction',1.))
        if factor not in (.9,1.,1.1):raise ValueError('Unsupported holdout friction')
        rig.model.geom_friction[np.r_[rig.geoms,rig.floor],0]*=factor
        walker=OmniCrawl(rig,task['params'])
        for command in task.get('commands',[task['direction']]):
            if command not in DIRECTIONS:raise ValueError('Unknown command')
            if 'command_params' in task:
                walker.p=dict(task['command_params'][command]);validate_params(walker.p)
            walker.segment(command,int(task['cycles']))
    except Exception:
        failure=traceback.format_exc();print(failure,flush=True)
    report=dict(passed=failure is None,simulation_only=True,hardware=False,algorithm='closed-loop crawl parameter search, not PPO',
        source=task['source'],params=task['params'],direction=task['direction'],cycles=task['cycles'],
        friction=task.get('friction',1.),failure=failure,wall_s=time.monotonic()-t0,
        segments=walker.segments if walker else [],steps=walker.steps if walker else [])
    if walker and walker.rows:
        np.savez_compressed(out/'trajectory.npz',qpos=np.asarray(walker.states),dt=.02)
        np.savez_compressed(out/'telemetry.npz',time=[x['time'] for x in walker.rows],
            position=[x['pos'] for x in walker.rows],clearance=[x['clearance'] for x in walker.rows],
            forces=[x['forces'] for x in walker.rows],controls=[x['ctrl'] for x in walker.rows],
            torques=[x['tau'] for x in walker.rows],yaw=[x['yaw'] for x in walker.rows],
            tilt=[x['tilt'] for x in walker.rows],power=[x['power'] for x in walker.rows])
        report['max_loop_gap_mm']=1000*max(x['loop_gap'] for x in walker.rows)
    write_json(out/'report.json',report);print('RESULT',report['passed'],task['direction'],flush=True)
    return report

if __name__=='__main__':
    if len(sys.argv)!=2:raise SystemExit('Worker only: use omni_parallel.py')
    execute(json.loads(Path(sys.argv[1]).read_text(encoding='utf-8')))
