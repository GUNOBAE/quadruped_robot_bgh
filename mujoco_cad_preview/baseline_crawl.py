"""Non-learning, free-body, gravity/contact-enabled linked-leg crawl. SIM ONLY.

Run both directions and replay the recorded physical test:
  .venv\\Scripts\\python.exe baseline_crawl.py --cycles 3 --view
Replay does not simulate walking: it displays the states recorded by mj_step.
No masses, motor gains/ranges, contact parameters or CAD files are changed.
"""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import time
import traceback
import zipfile
# Tiny dense systems should not fan out across all CPU cores. Process-local only.
os.environ["OPENBLAS_NUM_THREADS"]="1"
os.environ["OMP_NUM_THREADS"]="1"
import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation
import mujoco
from linkage_core import LinkageRig, LEGS, load_model, smooth

ROOT=Path(__file__).resolve().parent


def dump(path,obj):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(obj,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf-8')


class Crawl:
    def __init__(self,rig,cycles=3,stride=.012,lift=.015):
        self.r=rig;state=rig.settle()
        self.qref=rig.data.qpos.copy();self.anchors=state['feet'].copy()
        self.base=state['position'].copy();self.com_offset=state['com'][:2]-self.base[:2]
        self.integral=np.zeros(6);self.rows=[];self.states=[];self.steps=[];self.plans=[]
        self.cycles=cycles;self.stride=stride;self.lift=lift;self.ready_index=None
        self.height=.115;self.t=0.

    def center(self):
        return np.r_[self.anchors[:,:2].mean(axis=0)-self.com_offset,self.height]

    def support_pose(self,leg):
        """Finite quasi-static feasibility search; not RL and not a motor-limit change."""
        r=self.r;goals=self.anchors.copy();goals[leg,1]-=self.stride/2;goals[leg,2]+=self.lift
        stance=[i for i in range(4) if i!=leg];center=self.center();warm=self.qref.copy();values=[]
        def cost(p):
            if np.linalg.norm(p[:2]-center[:2])>.065 or not .09<p[2]<.14:
                return 100.+float(np.linalg.norm(p-center))
            q,error=r.inverse_kinematics(p,np.array([1.,0,0,0]),goals,warm)
            control,torque,normals,balance=r.load_compensation(q,stance)
            load_ratio=max(float(np.max(abs(control)/(.96*r.control_limit))),
                           float(np.max(abs(torque)/(.94*r.force_limit))))
            # ~8N minimum corner force for this ~7.16kg provisional source.
            minimum_force=.114*r.weight
            score=load_ratio+10*max(0.,minimum_force-min(normals))/r.weight+balance
            score+=.4*float(np.linalg.norm(p[:2]-center[:2]))
            values.append((float(score),p.copy(),load_ratio,float(min(normals)),
                           float(max(abs(control))),float(max(abs(torque)))))
            return score
        for dx in (-.025,-.0125,0.,.0125,.025):
            for dy in (-.02,0.,.02):
                for z in (.11,.115,.12):cost(center+[dx,dy,z-self.height])
        seed=min(values,key=lambda v:v[0])[1]
        minimize(cost,seed,method='Nelder-Mead',options={'maxiter':70,'xatol':.0002,'fatol':.0005})
        best=min(values,key=lambda v:v[0])
        plan=dict(leg=LEGS[leg],body_position_m=best[1].tolist(),load_ratio=best[2],
                  minimum_predicted_normal_N=best[3],max_reference_ctrl_rad=best[4],
                  max_predicted_torque_Nm=best[5])
        self.plans.append(plan)
        if best[2]>1.18 or best[3]<.04*r.weight:
            raise RuntimeError('No usable three-foot support pose: '+str(plan))
        print(f"SUPPORT {LEGS[leg]}: motor/load utilization={best[2]:.3f}, min support={best[3]:.1f}N",flush=True)
        return best[1]

    def control(self,body,goals,stance,phase,leg=-1,load_blend=1.):
        r=self.r;m,d=r.model,r.data
        R=d.xmat[r.base].reshape(3,3).copy()
        error=np.r_[body-d.xpos[r.base],-Rotation.from_matrix(R).as_rotvec()]
        self.integral+=r.dt*error
        self.integral[:3]=np.clip(self.integral[:3],-.008,.008)
        self.integral[3:]=np.clip(self.integral[3:],-.05,.05)
        virtual_body=body+.45*error[:3]+.6*self.integral[:3]
        rotation=Rotation.from_rotvec(.45*error[3:]+.6*self.integral[3:])
        quat=rotation.as_quat()[[3,0,1,2]]
        targets=goals.copy()
        # Swing feet track world targets using MEASURED base pose. Supporting
        # feet retain world anchors to stabilize the base via position servos.
        for i in range(4):
            if i not in stance:
                targets[i]=virtual_body+rotation.as_matrix()@R.T@(goals[i]-d.xpos[r.base])
        self.qref,ik_error=r.inverse_kinematics(virtual_body,quat,targets,self.qref)
        request,torque,normals,balance=r.load_compensation(self.qref,stance)
        if load_blend<1.:
            four,_,_,_=r.load_compensation(self.qref,[0,1,2,3])
            request=four*(1-load_blend)+request*load_blend
        # Only the 12 position-controller inputs modify the live robot.
        # No qpos rewriting, applied base force, gravity reduction or mass scaling.
        d.ctrl[:]=np.clip(d.ctrl+np.clip(request-d.ctrl,-.02,.02),r.low,r.high)
        before_warning=d.warning.number.copy();old_t=d.time
        for _ in range(r.substeps):mujoco.mj_step(m,d)
        state=r.snapshot();self.t+=r.dt
        if (np.any(d.warning.number>before_warning) or d.time<=old_t or
            not np.all(np.isfinite(d.qpos)) or not np.all(np.isfinite(d.qvel))):
            raise RuntimeError('MuJoCo numerical warning/reset: stop, do not call it walking.')
        row=dict(time_s=self.t,phase=phase,leg=leg,position_m=state['position'].tolist(),
                 foot_positions_m=state['feet'].tolist(),foot_normal_N=state['forces'].tolist(),
                 clearance_m=state['clearance'].tolist(),com_m=state['com'].tolist(),
                 tilt_deg=state['tilt'],loop_gap_m=state['loop_gap'],torso_contact=state['torso_contact'],
                 ik_error_m=ik_error,requested_ctrl_rad=request.tolist(),actual_ctrl_rad=d.ctrl.tolist(),
                 actual_motor_angle_rad=d.qpos[r.qadr].tolist(),motor_torque_Nm=d.actuator_force.tolist(),
                 ctrl_clipped_fraction=float(np.mean((request<r.low)|(request>r.high))),
                 torque_saturation_fraction=float(np.mean(abs(d.actuator_force)>=.99*r.force_limit)))
        self.rows.append(row);self.states.append(d.qpos.copy())
        if state['tilt']>25 or state['position'][2]<.075 or state['torso_contact'] or state['loop_gap']>.0015:
            raise RuntimeError(f"Unsafe simulated state: t={self.t:.2f}, tilt={state['tilt']:.1f}, h={state['position'][2]:.3f}")

    def shift(self,target,duration=2.,phase='SHIFT'):
        old=self.base.copy()
        for k in range(round(duration/.02)):
            p=old+(target-old)*smooth((k+1)*.02/duration)
            self.control(p,self.anchors,[0,1,2,3],phase)
        self.base=target.copy()

    def step_leg(self,leg):
        start_index=len(self.rows);initial=self.anchors.copy();target=initial[leg].copy()
        target[1]-=self.stride;peak=initial[leg,2]+self.lift
        stance=[i for i in range(4) if i!=leg]
        for k in range(125):
            t=(k+1)*.02;goals=initial.copy()
            if t<.5:
                blend=smooth(t/.5)
            elif t<1.1:
                goals[leg,2]+=self.lift*smooth((t-.5)/.6);blend=1.
            elif t<1.9:
                goals[leg,1]-=self.stride*smooth((t-1.1)/.8);goals[leg,2]=peak;blend=1.
            else:
                goals[leg]=target;goals[leg,2]=peak-self.lift*smooth((t-1.9)/.6);blend=1.
            self.control(self.base,goals,stance,'SWING',leg,blend)
        self.anchors[leg]=target
        for _ in range(35):self.control(self.base,self.anchors,[0,1,2,3],'LAND',leg)
        rows=self.rows[start_index:];lift_rows=[x for x in rows if x['phase']=='SWING']
        peak_clear=max(x['clearance_m'][leg] for x in lift_rows)
        airborne=sum(x['clearance_m'][leg]>.008 and x['foot_normal_N'][leg]<1. for x in lift_rows)*.02
        landed=float(np.mean([x['foot_normal_N'][leg] for x in rows[-10:]]))
        support=min(sum(x['foot_normal_N'][j]>2. for j in stance) for x in lift_rows[30:-20])
        target_error=float(np.linalg.norm(np.asarray(rows[-1]['foot_positions_m'][leg])-target))
        result=dict(leg=LEGS[leg],peak_clearance_mm=1000*peak_clear,airborne_seconds=airborne,
                    touchdown_normal_N=landed,minimum_other_support_feet=int(support),
                    final_placement_error_mm=target_error*1000,
                    passed=bool(airborne>=.2 and landed>2. and support==3 and target_error<.008))
        self.steps.append(result)
        print(f"STEP {LEGS[leg]}: clearance={1000*peak_clear:.1f}mm, airborne={airborne:.2f}s, touchdown={landed:.1f}N, pass={result['passed']}",flush=True)
        if not result['passed']:
            raise RuntimeError('Foot failed lift/support/touchdown gate; no more steps.')

    def run(self):
        self.shift(self.center(),phase='READY')
        self.ready_index=len(self.rows)-1
        for cycle in range(self.cycles):
            print(f'CYCLE {cycle+1}/{self.cycles}',flush=True)
            for leg in (0,3,1,2):
                self.shift(self.support_pose(leg));self.step_leg(leg)
        self.shift(self.center(),phase='FINISH')

    def report(self,failure=None):
        rows=self.rows;ready=self.ready_index
        start=np.asarray(rows[ready]['position_m']) if ready is not None else np.zeros(3)
        end=np.asarray(rows[-1]['position_m']) if rows else start
        forward=float(-(end-start)[1]);target=self.cycles*self.stride
        ratio=forward/target if target else 0.
        tilt=max((x['tilt_deg'] for x in rows),default=0.)
        passed=bool(failure is None and len(self.steps)==4*self.cycles and all(x['passed'] for x in self.steps)
                    and .6<=ratio<=1.4 and abs(end[0]-start[0])<.01 and tilt<10.)
        return dict(status='SIMULATED_CRAWL_PASS' if passed else 'FAILED_NO_WALKING_CLAIM',
            simulation_only=True,hardware_verified=False,failure=failure,
            physics_version=mujoco.__version__,gravity=self.r.model.opt.gravity.tolist(),
            mass_kg=float(self.r.model.body_mass.sum()),motor_range_unchanged=True,
            torque_limits_Nm=self.r.force_limit.tolist(),root_external_forces_used=False,
            measured_forward_mm=1000*forward,target_forward_mm=1000*target,
            lateral_drift_mm=1000*float(end[0]-start[0]),target_distance_ratio=ratio,
            max_tilt_deg=tilt,simulated_seconds=len(rows)*.02,
            max_loop_gap_mm=1000*max((x['loop_gap_m'] for x in rows),default=0.),
            mean_command_clipping_fraction=float(np.mean([x['ctrl_clipped_fraction'] for x in rows])) if rows else 0.,
            mean_torque_saturation_fraction=float(np.mean([x['torque_saturation_fraction'] for x in rows])) if rows else 0.,
            steps=self.steps,support_plans=self.plans,
            warnings=['Slow non-learning crawl, NOT a 40mm/s walking policy.',
                      'Full simulation state is used; not directly portable to IMU-only hardware.',
                      'Physical servo channel/neutral/polarity and mass/contact calibration remain unverified.',
                      'Only feet and torso have source collision geometry; not a self-collision proof.'])


def replay(run,direction='forward'):
    import mujoco.viewer
    model,_=load_model(Path(run)/'source.xml',visuals=True)
    data=mujoco.MjData(model)
    path=Path(run)/direction/'trajectory.npz'
    if not path.is_file():raise FileNotFoundError(str(path))
    saved=np.load(path,allow_pickle=False);states=saved['qpos'];dt=float(saved['dt'])
    report=json.loads((path.parent/'report.json').read_text('utf-8'))
    if report['status']!='SIMULATED_CRAWL_PASS':
        raise RuntimeError('Refusing to replay a failed run as walking. Read report.json.')
    print('RECORDED PHYSICS REPLAY:',report['status'],'Not a new simulation or learned policy.',flush=True)
    print('Recorded frames were generated with gravity/contact and position commands only. Close window to exit.',flush=True)
    with mujoco.viewer.launch_passive(model,data) as viewer:
        viewer.cam.distance=.58;viewer.cam.azimuth=145;viewer.cam.elevation=-22
        viewer.cam.lookat[:]=[0.,0.,.06]
        start=time.perf_counter();last_index=-1
        while viewer.is_running():
            i=int((time.perf_counter()-start)/dt)%len(states)
            if i!=last_index:
                with viewer.lock():
                    data.qpos[:]=states[i];data.qvel[:]=0.;mujoco.mj_forward(model,data)
                viewer.sync();last_index=i
            time.sleep(.005)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,default=ROOT/'output/physics_v1.xml')
    parser.add_argument('--cycles',type=int,default=3)
    parser.add_argument('--direction',choices=('forward','backward','both'),default='both')
    parser.add_argument('--view',action='store_true')
    parser.add_argument('--view-only',action='store_true')
    args=parser.parse_args()
    folder=ROOT/'output/baseline_crawl';folder.mkdir(parents=True,exist_ok=True)
    if args.view_only:
        run=Path((folder/'LATEST.txt').read_text('utf-8').strip())
        replay(run,'backward' if args.direction=='backward' else 'forward');return
    if not 1<=args.cycles<=6:parser.error('--cycles must be 1..6; no long learning jobs here.')
    if not args.source.is_file():raise FileNotFoundError('Existing Physics V1 source is missing: '+str(args.source))
    source_bytes=args.source.read_bytes();digest=hashlib.sha256(source_bytes).hexdigest()
    run=folder/time.strftime('%Y%m%d_%H%M%S');run.mkdir(exist_ok=False)
    (run/'source.xml').write_bytes(source_bytes)
    (folder/'LATEST.txt').write_text(str(run.resolve()),encoding='utf-8')
    results=[]
    for direction in (('forward','backward') if args.direction=='both' else (args.direction,)):
        output=run/direction;output.mkdir()
        walker=None;failure=None
        try:
            rig=LinkageRig(args.source)
            walker=Crawl(rig,args.cycles,.012 if direction=='forward' else -.012)
            walker.run()
        except Exception:
            failure=traceback.format_exc();(output/'error.txt').write_text(failure,encoding='utf-8');print(failure,flush=True)
        if walker is None:
            report=dict(status='FAILED_INITIALIZATION',failure=failure,simulation_only=True)
        else:
            report=walker.report(failure)
            if walker.rows:
                np.savez_compressed(output/'trajectory.npz',qpos=np.asarray(walker.states),dt=.02)
                dump(output/'telemetry.json',walker.rows)
        report['direction']=direction;report['source_sha256']=digest
        dump(output/'report.json',report);results.append(report)
        print('RESULT',direction,report['status'],'distance mm',report.get('measured_forward_mm'),flush=True)
    if hashlib.sha256(args.source.read_bytes()).hexdigest()!=digest:
        raise RuntimeError('Source was changed by another process during this run.')
    dump(run/'summary.json',dict(source=str(args.source.resolve()),source_sha256=digest,results=results,
         no_training=True,no_hardware_control=True,original_files_modified=False))
    with zipfile.ZipFile(run/'baseline_results.zip','w',zipfile.ZIP_DEFLATED) as z:
        for path in run.rglob('*'):
            if path.is_file() and path.name!='baseline_results.zip':z.write(path,str(path.relative_to(run)))
    print('REPORT:',run/'summary.json');print('SHARE:',run/'baseline_results.zip')
    print('SIMULATION ONLY. Original CAD, PPO checkpoints, masses and motor limits are unchanged.')
    passed=[r for r in results if r['status']=='SIMULATED_CRAWL_PASS']
    if args.view and passed:
        replay(run,passed[0]['direction'])
    elif args.view:
        print('No passed run to replay. Read report/error files; do not start PPO.')


if __name__=='__main__':
    main()
