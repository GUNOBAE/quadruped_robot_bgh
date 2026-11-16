"""Bounded timing/stride search over PR8 Crawl. No PPO or hardware control.

The existing nonlinear IK, load compensation, gains and motor limits are kept.
Mechanical work is a 50 Hz sampled |torque * joint velocity| proxy, not battery
energy: servo holding losses, electronics and motor efficiency are NOT included.
"""
from __future__ import annotations
import os
os.environ['OPENBLAS_NUM_THREADS'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'
import json
import math
import sys
import time
import traceback
from pathlib import Path
import numpy as np
from baseline_crawl import Crawl
from linkage_core import LinkageRig, LEGS, smooth

DEFAULT = dict(stride_mm=12., lift_mm=15., shift_s=2., unload_s=.5,
               swing_scale=1., land_s=.7)
BOUNDS = dict(stride_mm=(10.,20.), lift_mm=(12.,18.), shift_s=(.6,2.),
              unload_s=(.25,.5), swing_scale=(.4,1.), land_s=(.3,.7))


def write_json(path, value):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False), encoding='utf-8')
    tmp.replace(path)


def check_params(params):
    if set(params) != set(DEFAULT):
        raise ValueError('Unexpected parameter keys; physics/gains cannot be optimized here.')
    for k,(lo,hi) in BOUNDS.items():
        if not math.isfinite(float(params[k])) or not lo <= float(params[k]) <= hi:
            raise ValueError(f'{k} outside guarded range {lo}..{hi}')


class TimedCrawl(Crawl):
    def __init__(self, rig, params, direction, cycles):
        check_params(params)
        self.params = dict(params)
        sign = 1 if direction == 'forward' else -1
        super().__init__(rig, cycles=cycles, stride=sign*params['stride_mm']/1000,
                         lift=params['lift_mm']/1000)

    def control(self, *args, **kwargs):
        super().control(*args, **kwargs)
        r = self.r
        torque = r.data.actuator_force.copy()
        velocity = r.data.qvel[r.dadr].copy()
        self.rows[-1]['sampled_mechanical_power_W'] = float(np.sum(np.abs(torque*velocity)))
        self.rows[-1]['motor_velocity_rad_s'] = velocity.tolist()
        self.rows[-1]['torque_squared_sum'] = float(np.sum(torque**2))

    def shift(self, target, duration=None, phase='SHIFT'):
        # Initial and final centering retain the verified two-second duration.
        if duration is None:
            duration = self.params['shift_s'] if phase == 'SHIFT' else 2.
        super().shift(target, duration=duration, phase=phase)

    def step_leg(self, leg):
        p = self.params; dt = self.r.dt
        begin = len(self.rows); initial = self.anchors.copy()
        target = initial[leg].copy(); target[1] -= self.stride
        peak = initial[leg,2] + self.lift; stance = [i for i in range(4) if i != leg]
        # Preserve the original phase structure. Durations change, not support
        # constraints, motor strengths, physics integration rate or state feedback.
        unload = p['unload_s']; up = .6*p['swing_scale']
        travel = .8*p['swing_scale']; down = .6*p['swing_scale']
        total = unload + up + travel + down
        count = max(1, round(total/dt))
        for k in range(count):
            t = min(total,(k+1)*total/count); goals = initial.copy()
            if t < unload:
                blend = smooth(t/unload)
            elif t < unload+up:
                goals[leg,2] += self.lift*smooth((t-unload)/up); blend = 1.
            elif t < unload+up+travel:
                goals[leg,1] -= self.stride*smooth((t-unload-up)/travel)
                goals[leg,2] = peak; blend = 1.
            else:
                goals[leg] = target
                goals[leg,2] = peak-self.lift*smooth((t-unload-up-travel)/down); blend = 1.
            self.control(self.base,goals,stance,'SWING',leg,blend)
        self.anchors[leg] = target
        for _ in range(max(1, round(p['land_s']/dt))):
            self.control(self.base,self.anchors,[0,1,2,3],'LAND',leg)
        rows = self.rows[begin:]; swing = [r for r in rows if r['phase'] == 'SWING']
        air = [r for r in swing if r['clearance_m'][leg] > .008 and r['foot_normal_N'][leg] < 1.]
        air_time = len(air)*dt
        support = min((sum(r['foot_normal_N'][j] > 2. for j in stance) for r in air), default=0)
        landing = float(np.mean([r['foot_normal_N'][leg] for r in rows[-min(10,round(p['land_s']/dt)):]]))
        error = float(np.linalg.norm(np.asarray(rows[-1]['foot_positions_m'][leg])-target))
        result = dict(leg=LEGS[leg],peak_clearance_mm=1000*max(r['clearance_m'][leg] for r in swing),
                      airborne_seconds=air_time,touchdown_normal_N=landing,
                      minimum_other_support_feet=int(support),final_placement_error_mm=1000*error,
                      passed=bool(air_time >= .2 and landing > 2. and support == 3 and error < .008))
        self.steps.append(result)
        print('STEP',LEGS[leg],json.dumps(result),flush=True)
        if not result['passed']:
            raise RuntimeError('Lift / three-foot support / landing gate failed; reject candidate.')

    def report(self, failure=None):
        result = super().report(failure)
        measured = self.rows[(self.ready_index+1):] if self.ready_index is not None else []
        duration = len(measured)*self.r.dt
        distance = abs(result['measured_forward_mm'])/1000
        work = sum(x['sampled_mechanical_power_W'] for x in measured)*self.r.dt
        cot = work/(self.r.weight*distance) if distance > 1e-6 else None
        # Aggregate trial score is never a replacement for the original gates.
        result.update(params=self.params,measured_window_s=duration,
                      speed_mm_s=1000*distance/max(duration,1e-9),
                      sampled_abs_mechanical_work_J=work,
                      mechanical_cot_proxy=cot,
                      mean_sum_torque_squared=float(np.mean([r['torque_squared_sum'] for r in measured])) if measured else None,
                      energy_warning='50Hz sampled mechanical work proxy, not electrical consumption; static holding loss excluded.',
                      learning_type='bounded gait parameter optimization, NOT PPO',
                      original_controller='PR8 nonlinear linkage IK and load transfer')
        return result


def worker(task_path):
    task = json.loads(Path(task_path).read_text('utf-8'))
    out = Path(task['out']); out.mkdir(parents=True,exist_ok=True)
    check_params(task['params'])
    started = time.monotonic(); reports = []
    for direction in ('forward','backward'):
        target = out/direction; target.mkdir(exist_ok=True)
        walker = None; failure = None
        try:
            rig = LinkageRig(Path(task['source']))
            factor = float(task.get('friction_factor',1.))
            if factor not in (.9,1.,1.1):
                raise ValueError('Only nominal or explicit +/-10% friction holdout is supported.')
            if factor != 1.:
                # Holdout evaluation only. Never modify source XML or nominal trials.
                ids = np.r_[rig.geoms,rig.floor]
                rig.model.geom_friction[ids,0] *= factor
            if task.get('original_gate'):
                # Literal original controller runs first. No timing override.
                walker = Crawl(rig,cycles=task['cycles'],stride=.012 if direction=='forward' else -.012)
            else:
                walker = TimedCrawl(rig,task['params'],direction,task['cycles'])
            walker.run()
        except Exception:
            failure = traceback.format_exc()
            (target/'error.txt').write_text(failure,encoding='utf-8')
            print(failure,flush=True)
        result = walker.report(failure) if walker is not None else dict(status='FAILED_INITIALIZATION',failure=failure)
        result.update(direction=direction,friction_factor=task.get('friction_factor',1.),cycles=task['cycles'])
        write_json(target/'report.json',result); reports.append(result)
        if walker is not None and walker.rows:
            np.savez_compressed(target/'trajectory.npz',qpos=np.asarray(walker.states),dt=.02)
            # Compact telemetry, enough to verify contact cycles and energy.
            np.savez_compressed(target/'telemetry.npz',
                time=np.asarray([r['time_s'] for r in walker.rows]),
                foot_normal=np.asarray([r['foot_normal_N'] for r in walker.rows]),
                clearance=np.asarray([r['clearance_m'] for r in walker.rows]),
                controls=np.asarray([r['actual_ctrl_rad'] for r in walker.rows]),
                angles=np.asarray([r['actual_motor_angle_rad'] for r in walker.rows]),
                torques=np.asarray([r['motor_torque_Nm'] for r in walker.rows]),
                power=np.asarray([r.get('sampled_mechanical_power_W',0.) for r in walker.rows]))
        if result['status'] != 'SIMULATED_CRAWL_PASS':
            break  # Reject quickly; never waste a second direction on a failed first.
    ok = len(reports)==2 and all(r['status']=='SIMULATED_CRAWL_PASS' for r in reports)
    summary = dict(passed=ok,reports=reports,params=task['params'],cycles=task['cycles'],
                   friction_factor=task.get('friction_factor',1.),elapsed_wall_s=time.monotonic()-started,
                   original_gate=task.get('original_gate',False))
    write_json(out/'trial.json',summary)
    print('TRIAL FINISHED:', 'PASS' if ok else 'REJECTED',flush=True)


if __name__ == '__main__':
    if len(sys.argv)!=2:
        raise SystemExit('Internal worker: pass a task.json path. Use crawl_night.py to start a batch.')
    worker(sys.argv[1])
