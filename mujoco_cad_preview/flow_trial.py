"""Overlapping body/foot trajectories. SIM ONLY, parameter search not PPO.
Uses original linkage geometry, mass, position actuators and force limits.
Planning may assign qpos only on the separate IK planner, never on live data.
"""
from __future__ import annotations
import os
for _k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[_k]='1'
import json, math, sys, time, traceback
from pathlib import Path
import numpy as np
from omni_trial import OmniCrawl, DIRECTIONS, DEFAULT as OLD, write_json
from linkage_core import LinkageRig, LEGS

DEFAULT = dict(stride_mm=20.,lift_mm=15.,slot_s=1.6,swing_fraction=.6,
               drift=.3,travel_weight=0.,forward_bias=0.,curve=5)
BOUNDS = dict(stride_mm=(8.,24.),lift_mm=(13.,18.),slot_s=(.9,2.4),
              swing_fraction=(.45,.70),drift=(.1,.7),travel_weight=(0.,.15),forward_bias=(0.,.5),curve=(5,5))
TRANSLATIONS=tuple(k for k,v in DIRECTIONS.items() if k!='stop' and v[2]==0)


def valid(p):
    if set(p)!=set(DEFAULT):raise ValueError('Unexpected parameter keys')
    for k,(lo,hi) in BOUNDS.items():
        if not math.isfinite(float(p[k])) or not lo<=p[k]<=hi:raise ValueError('Invalid '+k)


def hermite5(p0,p1,v0,v1,T,u):
    """Quintic position with prescribed endpoint velocities, zero endpoint acceleration."""
    s=float(np.clip(u,0.,1.));s3=s**3;s4=s**4;s5=s**5
    return (1-10*s3+15*s4-6*s5)*p0+(10*s3-15*s4+6*s5)*p1+T*((s-6*s3+8*s4-3*s5)*v0+(-4*s3+7*s4-3*s5)*v1)


def metrics(w,start,initial,direction,cycles,stride):
    rows=w.rows[start:];dt=w.r.dt
    if len(rows)<6:raise ValueError('Too few measured samples')
    vector=np.array([DIRECTIONS[direction][1],-DIRECTIONS[direction][0]])
    normal=np.array([-vector[1],vector[0]])
    xyz=np.array([x['pos'] for x in rows]);xy=xyz[:,:2]-initial[:2]
    along=xy@vector;dist=float(along[-1]);dur=len(rows)*dt
    # Windowed velocities distinguish real stops/reversal from tiny servo jitter.
    lag=5
    vel=(along[lag:]-along[:-lag])/(dt*lag)
    idle=abs(vel)<.001 # fixed 1 mm/s threshold; reported, not relabelled as success
    max_idle=streak=0
    for x in idle:
        streak=streak+1 if x else 0;max_idle=max(max_idle,streak)
    vv=np.diff(xyz[:,:2],axis=0)/dt
    acc=np.diff(vv,axis=0)/dt
    work=sum(x['power'] for x in rows)*dt
    lift=[sum(x['forces'][i]<1 and x['clearance'][i]>.008 for x in rows)*dt for i in range(4)]
    return dict(duration_s=dur,net_mm=dist*1000,speed_mm_s=1000*dist/dur,
        target_mm=cycles*stride*1000,progress_ratio=dist/(cycles*stride),
        forward_fraction=float(np.mean(vel>.001)),reverse_fraction=float(np.mean(vel<-.001)),
        stall_fraction=float(np.mean(idle)),longest_stall_s=max_idle*dt,
        lateral_sway_mm=float(np.ptp(xy@normal)*1000),cross_drift_mm=float(abs(xy[-1]@normal)*1000),
        max_tilt_deg=max(x['tilt'] for x in rows),accel_rms=float(np.sqrt(np.mean(np.sum(acc*acc,axis=1)))),
        height_excursion_mm=float(np.ptp(xyz[:,2])*1000),absolute_mechanical_work_J=work,
        work_per_distance=work/max(dist,1e-6),work_warning='50Hz absolute mechanical power proxy; NOT battery energy',
        clip_fraction=float(np.mean([x['clip'] for x in rows])),force_sat_fraction=float(np.mean([x['force_sat'] for x in rows])),
        max_loop_gap_mm=max(x['loop_gap'] for x in rows)*1000,foot_airborne_s=lift)


class FlowCrawl(OmniCrawl):
    def __init__(self,r,p):
        valid(p);self.flow=dict(p)
        super().__init__(r,dict(OLD,stride_mm=min(p['stride_mm'],18.),lift_mm=p['lift_mm'],travel_weight=p['travel_weight'],curve=5))
        self.progress_each_cycle=[]

    def support_pose(self,leg,landing):
        if self.flow['forward_bias']==0 or not hasattr(self,'expected_body'):
            return super().support_pose(leg,landing)
        from scipy.optimize import minimize
        r=self.r;goals=self.anchors.copy();goals[leg]=(goals[leg]+landing)/2
        goals[leg,2]+=self.flow['lift_mm']/1000;stance=[i for i in range(4) if i!=leg]
        center=self.center();warm=self.qref.copy();values=[]
        def cost(pos):
            if np.linalg.norm(pos[:2]-center[:2])>.065 or not .09<pos[2]<.14:return 100.
            try:q,_=r.inverse_kinematics(pos,[1.,0,0,0],goals,warm)
            except RuntimeError:return 100.
            ctrl,tau,normals,balance=r.load_compensation(q,stance)
            use=max(float(np.max(abs(ctrl)/(.96*r.control_limit))),float(np.max(abs(tau)/(.94*r.force_limit))))
            score=use+10*max(0.,.114*r.weight-min(normals))/r.weight+balance
            score+=.4*np.linalg.norm(pos[:2]-center[:2])+self.flow['travel_weight']*np.linalg.norm(pos[:2]-self.base[:2])/.04
            score+=self.flow['forward_bias']*float((pos-self.expected_body)@self.heading/.02)**2
            values.append((float(score),pos.copy(),use,float(min(normals))))
            return float(score)
        for x in (-.025,-.0125,0.,.0125,.025):
            for y in (-.02,0.,.02):
                for h in (.11,.115,.12):cost(center+np.array([x,y,h-self.height]))
        if not values:raise RuntimeError('No support plan')
        minimize(cost,min(values,key=lambda a:a[0])[1],method='Nelder-Mead',options={'maxiter':70,'xatol':.0002,'fatol':.0005})
        best=min(values,key=lambda a:a[0])
        if best[2]>1.18 or best[3]<.04*r.weight:raise RuntimeError('Biased support failed load gate')
        self.plans.append(dict(leg=LEGS[leg],position=best[1].tolist(),use=best[2],min_force_N=best[3]))
        return best[1]

    def run_flow(self,direction,cycles):
        if direction not in TRANSLATIONS:raise ValueError('This experiment targets translation continuity, not yaw')
        self.command=direction;p=self.flow;stride=p['stride_mm']/1000;slot=p['slot_s'];duty=p['swing_fraction']
        vector=np.array([DIRECTIONS[direction][1],-DIRECTIONS[direction][0],0.])
        self.shift(self.center(),2.,'READY');start=len(self.rows);initial=self.r.data.xpos[self.r.base].copy()
        anchors=self.anchors.copy();body0=self.base.copy();q0=self.qref.copy();order=(0,3,1,2)
        poses=[];landings=[]
        # Look ahead over all requested footfalls. Support poses are geometric
        # plans, not live-state edits; source model dynamics remain unchanged.
        for j in range(cycles*4):
            leg=order[j%4];target=self.anchors[leg]+vector*stride
            self.heading=vector;self.expected_body=body0+vector*stride*(j+.5)/4
            pose=self.support_pose(leg,target);poses.append(pose.copy());landings.append(target.copy())
            self.base=pose.copy();self.anchors[leg]=target
            self.qref,_=self.r.inverse_kinematics(pose,[1.,0,0,0],self.anchors,self.qref)
        self.anchors=anchors;self.base=body0;self.qref=q0
        lo=(1-duty)/2;hi=1-lo;velocity=vector*stride*p['drift']/slot
        ts=[0.];points=[body0];velocities=[np.zeros(3)]
        for j,pose in enumerate(poses):
            ts += [(j+lo)*slot,(j+hi)*slot]
            points += [pose-velocity*duty*slot/2,pose+velocity*duty*slot/2]
            velocities += [velocity,velocity]
        ts += [len(poses)*slot];points += [poses[-1]+velocity*slot*.5];velocities += [np.zeros(3)]
        ts=np.array(ts);points=np.array(points);velocities=np.array(velocities)
        def body_path(t):
            i=min(len(ts)-2,max(0,int(np.searchsorted(ts,t,side='right'))-1));T=ts[i+1]-ts[i]
            return hermite5(points[i],points[i+1],velocities[i],velocities[i+1],T,(t-ts[i])/T)
        last_cycle=self.r.data.xpos[self.r.base].copy()
        for j,target in enumerate(landings):
            leg=order[j%4];old=self.anchors[leg].copy();stance=[i for i in range(4) if i!=leg];begin=len(self.rows)
            count=max(1,round(slot/self.r.dt))
            for k in range(count):
                ph=(k+1)/count;body=body_path((j+ph)*slot);goals=self.anchors.copy()
                if lo<=ph<=hi:
                    u=(ph-lo)/duty
                    goals[leg]=old+(target-old)*self.easing(u)
                    goals[leg,2]+=p['lift_mm']/1000*math.sin(math.pi*u)**2
                    support=stance;blend=1.;phase='OVERLAPPED_SWING'
                elif ph<lo:
                    blend=self.easing((ph-lo+.12)/.12);support=stance if blend>0 else [0,1,2,3];phase='TRANSFER'
                else:
                    goals[leg]=target;blend=1-self.easing((ph-hi)/.12)
                    support=stance if blend>0 else [0,1,2,3];phase='LAND'
                self.control(body,goals,support,phase,leg,blend)
            self.anchors[leg]=target;self.base=body.copy()
            rows=self.rows[begin:];air=[x for x in rows if x['clearance'][leg]>.008 and x['forces'][leg]<1.]
            nlast=min(6,len(rows));contact=float(np.mean([x['forces'][leg] for x in rows[-nlast:]]))
            supporting=min((sum(x['forces'][i]>2. for i in stance) for x in air),default=0)
            err=float(np.linalg.norm(np.array(rows[-1]['feet'][leg])-target))
            step=dict(leg=LEGS[leg],air_s=len(air)*self.r.dt,touchdown_N=contact,support=int(supporting),error_mm=err*1000,
                      passed=bool(len(air)*self.r.dt>=.20 and contact>2 and supporting==3 and err<.008))
            self.steps.append(step);print('STEP',j+1,step,flush=True)
            if not step['passed']:raise RuntimeError('Overlap lift/support/placement gate failed')
            if (j+1)%4==0:
                now=self.r.data.xpos[self.r.base].copy()
                progress=float((now-last_cycle)@vector/stride);self.progress_each_cycle.append(progress);last_cycle=now
                if progress<.4:raise RuntimeError('One complete cycle made insufficient progress')
        self.shift(self.center(),1.,'FINISH')
        return start,initial,stride


def execute(task):
    out=Path(task['out']);out.mkdir(parents=True,exist_ok=True);w=None;error=None;result=None;beg=time.monotonic()
    direction=task['direction'];cycles=int(task['cycles'])
    try:
        if not 1<=cycles<=20 or direction not in TRANSLATIONS:raise ValueError('Unsupported trial scope')
        rig=LinkageRig(task['source']);mu=float(task.get('friction',1.))
        if mu not in (.9,1.,1.1):raise ValueError('Unsupported holdout')
        rig.model.geom_friction[np.r_[rig.geoms,rig.floor],0]*=mu
        if task.get('reference'):
            w=OmniCrawl(rig,task['params']);w.shift(w.center(),2.,'READY');w.prepared=True
            start=len(w.rows);initial=rig.data.xpos[rig.base].copy();w.segment(direction,cycles)
            stride=task['params']['stride_mm']/1000
        else:
            w=FlowCrawl(rig,task['params']);start,initial,stride=w.run_flow(direction,cycles)
        result=metrics(w,start,initial,direction,cycles,stride)
        result['passed']=bool(.6<=result['progress_ratio']<=1.4 and result['cross_drift_mm']<10
            and result['max_tilt_deg']<10 and result['clip_fraction']<.12 and result['force_sat_fraction']<.05)
        if not result['passed']:raise RuntimeError('Direction/posture/actuator gate failed')
    except Exception:error=traceback.format_exc();print(error,flush=True)
    report=dict(passed=error is None,metrics=result,failure=error,direction=direction,cycles=cycles,
        params=task['params'],reference=bool(task.get('reference')),friction=task.get('friction',1.),
        simulation_only=True,PPO=False,source_parameters_modified=False,wall_s=time.monotonic()-beg,
        steps=w.steps if w else [])
    if w and w.rows:
        np.savez_compressed(out/'trajectory.npz',qpos=np.array(w.states),dt=w.r.dt)
        np.savez_compressed(out/'telemetry.npz',time=[x['time'] for x in w.rows],position=[x['pos'] for x in w.rows],
            forces=[x['forces'] for x in w.rows],clearance=[x['clearance'] for x in w.rows],
            controls=[x['ctrl'] for x in w.rows],tau=[x['tau'] for x in w.rows],tilt=[x['tilt'] for x in w.rows])
    write_json(out/'report.json',report);print('RESULT',report['passed'],direction,result,flush=True);return report

if __name__=='__main__':execute(json.loads(Path(sys.argv[1]).read_text(encoding='utf-8-sig')))
