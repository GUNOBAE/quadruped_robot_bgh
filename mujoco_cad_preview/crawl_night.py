"""Local, time-bounded optimization of the working PR8 crawl, NOT PPO.

Start: python crawl_night.py --hours 8
Replay: python crawl_night.py --view-best --direction forward
Stop: Ctrl+C, or create STOP in the printed run directory.
All original physics/model/weights remain unchanged. No hardware I/O.
"""
from __future__ import annotations
import os
os.environ['OPENBLAS_NUM_THREADS'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'
import argparse
import csv
import ctypes
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import random
import shutil
import subprocess
import sys
import time
import traceback
import zipfile
from crawl_tune_trial import DEFAULT, BOUNDS, write_json

ROOT = Path(__file__).resolve().parent
FILES = ('linkage_core.py','baseline_crawl.py','crawl_tune_trial.py','crawl_night.py')


def read(path): return json.loads(Path(path).read_text(encoding='utf-8'))
def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def key(p): return tuple(round(p[k],4) for k in DEFAULT)


def ratios(trial, base):
    """Compare equal-cycle trials, both directions; no reward score involved."""
    if not trial or not trial.get('passed') or not base.get('passed'): return None
    if trial['cycles'] != base['cycles'] or trial['friction_factor'] != base['friction_factor']:
        raise ValueError('Cannot compare unlike cycle counts/contact conditions.')
    speed=[]; cost=[]; stable=True
    for a,b in zip(trial['reports'],base['reports']):
        if a['direction'] != b['direction']: raise ValueError('Direction mismatch')
        if a.get('mechanical_cot_proxy') is None or not b.get('mechanical_cot_proxy'): return None
        speed.append(a['speed_mm_s']/max(b['speed_mm_s'],1e-9))
        cost.append(a['mechanical_cot_proxy']/b['mechanical_cot_proxy'])
        stable &= (a['max_tilt_deg'] <= min(10.,b['max_tilt_deg']+3.)
                   and a['mean_command_clipping_fraction'] <= max(.08,b['mean_command_clipping_fraction']+.03)
                   and a['mean_torque_saturation_fraction'] <= max(.03,b['mean_torque_saturation_fraction']+.015))
    return dict(min_speed_ratio=min(speed),max_work_per_distance_ratio=max(cost),
                speed_ratios=speed,work_ratios=cost,stable=bool(stable),
                improved=bool(stable and min(speed)>=1.10 and max(cost)<=1.05),
                score=min(speed)-.5*max(0.,max(cost)-1.))


def launch(run,name,params,cycles,deadline,friction=1.,original=False):
    out=run/name
    if (out/'trial.json').is_file(): return read(out/'trial.json')
    if time.monotonic() >= deadline-5: return None
    if (run/'STOP').exists(): raise KeyboardInterrupt('STOP file found')
    manifest=read(run/'manifest.json')
    if sha(run/'source.xml') != manifest['source_sha256']:
        raise RuntimeError('Snapshot changed; stopping instead of comparing different physics.')
    out.mkdir(parents=True,exist_ok=True)
    task=dict(source=str(run/'source.xml'),params=params,cycles=cycles,
              out=str(out),friction_factor=friction,original_gate=original)
    write_json(out/'task.json',task)
    print('START',name,params,'cycles=',cycles,'friction=',friction,flush=True)
    # Isolated child: a hung/failed solver cannot eat the whole overnight budget.
    limit=min(deadline,time.monotonic()+1800.)
    with (out/'console.log').open('w',encoding='utf-8') as log:
        proc=subprocess.Popen([sys.executable,'-u',str(run/'code/crawl_tune_trial.py'),str(out/'task.json')],
                              cwd=run/'code',stdout=log,stderr=subprocess.STDOUT)
        try:
            while proc.poll() is None:
                if (run/'STOP').exists(): raise KeyboardInterrupt('STOP file found')
                if time.monotonic()>limit:
                    proc.terminate()
                    try: proc.wait(timeout=5)
                    except subprocess.TimeoutExpired: proc.kill();proc.wait()
                    write_json(out/'worker_error.json',dict(status='TIMEOUT',wall_limit_s=1800))
                    print('TIMEOUT:',name,flush=True);return None
                time.sleep(.2)
        except BaseException:
            if proc.poll() is None:
                proc.terminate()
                try: proc.wait(timeout=5)
                except subprocess.TimeoutExpired: proc.kill();proc.wait()
            raise
    if not (out/'trial.json').is_file():
        print('WORKER ERROR:',name,(out/'console.log').read_text('utf-8')[-1500:],flush=True)
        return None
    r=read(out/'trial.json');r['run_dir']=str(out)
    write_json(out/'trial.json',r)
    print('END',name,'PASS' if r['passed'] else 'REJECTED',flush=True)
    return r


def candidate_stream(seed):
    """Controlled one-variable probes first; caller then proposes bounded local trials."""
    probes=[{'shift_s':x} for x in (1.6,1.2,.9,.6)]
    probes += [{'swing_scale':x} for x in (.85,.7,.55,.4)]
    probes += [{'stride_mm':x} for x in (14.,16.,18.,20.)]
    probes += [{'lift_mm':x} for x in (12.,13.5,17.)]
    probes += [{'unload_s':.35},{'land_s':.4}]
    probes += [dict(shift_s=1.,swing_scale=.7,unload_s=.35,land_s=.5)]
    return [dict(DEFAULT,**p) for p in probes]


def summarize(run,status,screen,finals,selected):
    write_json(run/'status.json',dict(status=status,screened=len(screen),
                                    validated=len(finals),saved_at=time.time()))
    write_json(run/'screening.json',screen);write_json(run/'validation.json',finals)
    write_json(run/'selection.json',selected)
    cols=['trial','passed','min_speed_ratio','max_work_per_distance_ratio','stable','improved']
    with (run/'ranking.csv').open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=cols);w.writeheader()
        for x in sorted(screen,key=lambda a:a.get('comparison',{}).get('score',-1e6),reverse=True):
            c=x.get('comparison') or {}
            w.writerow(dict(trial=x['trial'],passed=x['passed'],**{k:c.get(k,'') for k in cols[2:]}))
    text=['# Overnight crawl parameter optimization', '', 'Status: '+status,
          'Type: controller parameter search. NOT PPO / NOT a new neural policy.',
          'Source SHA256: '+read(run/'manifest.json')['source_sha256'],
          '', 'Recommended result: '+selected.get('label','NONE'),
          'Recommended directory: '+selected.get('run_dir','NONE'),
          '', 'Acceptance: both directions retain lift/three-foot support/landing gates;',
          'at least 10% faster and <=5% worse mechanical-work-per-distance proxy;',
          'three-cycle validation, longer confirmation and +/-10% friction holdouts.',
          'Work is 50Hz sampled sum(abs(torque*velocity)); NOT battery power.',
          'Static holding current, driver losses and motor efficiency are not modeled.',
          'A faster replay at 1x is evidence of this simulation only, not real hardware.',
          '', 'Old files/weights are untouched. No gait -> no long PPO continuation.',
          'See ranking.csv, validation.json and each trial/report.json.']
    (run/'SUMMARY.md').write_text('\n'.join(text),encoding='utf-8')
    # Keep the shared archive compact; full states remain on the user's PC.
    temp=run/'results_bundle.tmp.zip'
    with zipfile.ZipFile(temp,'w',zipfile.ZIP_DEFLATED) as z:
        for p in run.rglob('*'):
            if p.is_file() and (p.suffix in ('.json','.md','.csv','.xml','.py') or p.name=='error.txt'):
                z.write(p,str(p.relative_to(run)))
    temp.replace(run/'results_bundle.zip')


def acquire_lock(folder):
    folder.mkdir(parents=True,exist_ok=True)
    handle=(folder/'RUNNING.lock').open('a+b');handle.seek(0)
    if handle.read(1)==b'':handle.write(b'0');handle.flush()
    handle.seek(0)
    try:
        if os.name=='nt':
            import msvcrt
            msvcrt.locking(handle.fileno(),msvcrt.LK_NBLCK,1)
        else:
            import fcntl
            fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except OSError:
        handle.close();raise RuntimeError('An overnight crawl experiment is already running. Do not start two.')
    return handle


def replay_selected(folder,direction):
    import numpy as np
    import mujoco
    import mujoco.viewer
    from linkage_core import load_model
    run=Path((folder/'LATEST.txt').read_text('utf-8').strip());selection=read(run/'selection.json')
    chosen=selection.get('run_dir')
    if not chosen:raise RuntimeError('No passed baseline/candidate is available. Read SUMMARY.md.')
    report=read(Path(chosen)/direction/'report.json')
    if report['status']!='SIMULATED_CRAWL_PASS':raise RuntimeError('Refusing to present a failed run as walking.')
    model,_=load_model(run/'source.xml',visuals=True);data=mujoco.MjData(model)
    with np.load(Path(chosen)/direction/'trajectory.npz',allow_pickle=False) as npz:
        states=npz['qpos'].copy();dt=float(npz['dt'])
    print('RESULT:',selection['label'],flush=True)
    print('1x recorded physics replay. Not new physics, not a trained PPO policy.',flush=True)
    print('Speed mm/s:',report.get('speed_mm_s'),'work proxy:',report.get('mechanical_cot_proxy'),flush=True)
    with mujoco.viewer.launch_passive(model,data) as viewer:
        viewer.cam.distance=.58;viewer.cam.azimuth=145;viewer.cam.elevation=-22
        viewer.cam.lookat[:]=[0,0,.06];start=time.perf_counter();last=-1
        while viewer.is_running():
            i=int((time.perf_counter()-start)/dt)%len(states)
            if i!=last:
                with viewer.lock():
                    data.qpos[:]=states[i];data.qvel[:]=0.;mujoco.mj_forward(model,data)
                viewer.sync();last=i
            time.sleep(.005)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--hours',type=float,default=8.)
    ap.add_argument('--max-trials',type=int,default=1000)
    ap.add_argument('--seed',type=int,default=20261117)
    ap.add_argument('--source',type=Path,default=ROOT/'output/physics_v1.xml')
    ap.add_argument('--view-best',action='store_true')
    ap.add_argument('--direction',choices=('forward','backward'),default='forward')
    ap.add_argument('--quick-test',action='store_true',help='Controller/batch test; NOT overnight optimization.')
    a=ap.parse_args()
    if not .1<=a.hours<=12:ap.error('--hours must be 0.1..12')
    if not 1<=a.max_trials<=1000:ap.error('--max-trials must be 1..1000')
    folder=ROOT/'output/crawl_night'
    if a.view_best:replay_selected(folder,a.direction);return
    if not a.source.is_file():raise FileNotFoundError(str(a.source))
    for f in FILES:
        if not (ROOT/f).is_file():raise FileNotFoundError(f+' missing; use the provided download command.')
    lock=acquire_lock(folder)
    run=folder/(time.strftime('%Y%m%d_%H%M%S')+f'_{os.getpid()}');run.mkdir()
    (folder/'LATEST.txt').write_text(str(run.resolve()),encoding='utf-8')
    (run/'code').mkdir()
    for f in FILES:shutil.copy2(ROOT/f,run/'code'/f)
    shutil.copy2(a.source,run/'source.xml')
    write_json(run/'manifest.json',dict(source_original=str(a.source.resolve()),source_sha256=sha(a.source),
        source_code_sha256={f:sha(ROOT/f) for f in FILES},max_hours=a.hours,seed=a.seed,
        versions={k:importlib.metadata.version(k) for k in ('mujoco','numpy','scipy')},
        algorithm='bounded direct parameter search; controlled probes then elite/local + global random proposals',
        gain_mass_limit_changes=False,simulation_only=True,hardware_control=False,
        bounds=BOUNDS,default=DEFAULT))
    start=time.monotonic();deadline=start+a.hours*3600
    reserve=min(3600.,max(120.,a.hours*3600*.25));search_deadline=deadline-reserve
    screen=[];finals=[];selected={};status='STARTING';awake=False
    print('CRAWL NIGHT STARTED:',run,flush=True)
    print('PARAMETER OPTIMIZATION, NOT PPO. Original crawl must pass first.',flush=True)
    try:
        if os.name=='nt':
            awake=bool(ctypes.windll.kernel32.SetThreadExecutionState(0x80000001))
            print('Windows automatic sleep inhibition:',awake,flush=True)
        nref=1 if a.quick_test else 3
        original=launch(run,'00_original_gate',DEFAULT,nref,deadline,original=True)
        if not original or not original['passed']:
            status='ORIGINAL_BASELINE_FAILED_NO_SEARCH';return
        ref3=launch(run,'01_reference',DEFAULT,nref,deadline)
        if not ref3 or not ref3['passed']:
            status='TIMED_BASELINE_FAILED_NO_SEARCH';return
        selected=dict(label='BASELINE_ONLY_NO_CONFIRMED_IMPROVEMENT',run_dir=ref3['run_dir'],params=DEFAULT)
        # Default timing must reproduce the unmodified controller, not merely survive.
        for x,y in zip(ref3['reports'],original['reports']):
            if abs(x['measured_forward_mm']-y['measured_forward_mm'])>2.:
                status='REFERENCE_REPRODUCTION_MISMATCH';return
        ref1=ref3 if nref==1 else launch(run,'02_reference_screen',DEFAULT,1,deadline)
        if not ref1 or not ref1['passed']:
            status='SCREEN_BASELINE_FAILED';return
        print('BASELINE VERIFIED. STARTING TIMING/STRIDE SEARCH.',flush=True)
        summarize(run,'SEARCHING',screen,finals,selected)
        rng=random.Random(a.seed);probes=candidate_stream(a.seed);seen={key(DEFAULT)}
        elite=[(0.,dict(DEFAULT))];stale=0;best=-1e6;errors=0
        count=1 if a.quick_test else a.max_trials
        for i in range(count):
            if not a.quick_test and time.monotonic()>search_deadline:break
            if i<len(probes):params=probes[i]
            else:
                for attempt in range(200):
                    if i%5==0:
                        params={k:rng.uniform(*v) for k,v in BOUNDS.items()}
                    else:
                        parent=rng.choice(elite[:5])[1];params=dict(parent)
                        keys=rng.sample(list(DEFAULT),k=1 if i%3 else 2)
                        for k in keys:
                            lo,hi=BOUNDS[k];width=(hi-lo)*max(.06,.25/(1+i/80))
                            params[k]=min(hi,max(lo,params[k]+rng.gauss(0,width)))
                    params={k:round(v,4) for k,v in params.items()}
                    if key(params) not in seen:break
                else:status='SEARCH_SPACE_EXHAUSTED';break
            seen.add(key(params))
            name=f'trial_{i:04d}'
            result=launch(run,name,params,1,deadline if a.quick_test else search_deadline)
            if result is None:
                errors+=1
                if errors>=3:status='THREE_WORKER_ERRORS';break
                continue
            errors=0;cmp=ratios(result,ref1)
            row=dict(trial=name,passed=result['passed'],params=params,run_dir=result['run_dir'],comparison=cmp)
            screen.append(row)
            if cmp and cmp['stable']:
                elite.append((cmp['score'],params));elite.sort(key=lambda x:x[0],reverse=True)
                elite=elite[:10]
                if cmp['score']>best+.01:best=cmp['score'];stale=0
                else:stale+=1
                print('COMPARE speed x',round(cmp['min_speed_ratio'],3),'work/distance x',round(cmp['max_work_per_distance_ratio'],3),flush=True)
            else:stale+=1
            summarize(run,'SEARCHING',screen,finals,selected)
            if i>=79 and stale>=80:
                status='SEARCH_PLATEAU';break
        # Finalists are compared against the same-cycle reference, then held out.
        candidates=[r for r in screen if r.get('comparison') and r['comparison']['improved']]
        candidates.sort(key=lambda r:r['comparison']['score'],reverse=True)
        winners=[]
        for i,c in enumerate(candidates[:5]):
            if time.monotonic()>deadline-60:break
            result=launch(run,f'validation_{i}',c['params'],nref,deadline)
            cmp=ratios(result,ref3) if result else None
            record=dict(trial=c['trial'],params=c['params'],comparison=cmp,validation_dir=result.get('run_dir') if result else None)
            finals.append(record)
            if cmp and cmp['improved']:
                confirmed=True;holdouts=[]
                for factor in (() if a.quick_test else (.9,1.1)):
                    refhold=launch(run,f'holdout_reference_{factor}',DEFAULT,3,deadline,friction=factor)
                    hold=launch(run,f'holdout_{i}_{factor}',c['params'],3,deadline,friction=factor)
                    comparison=ratios(hold,refhold) if hold and refhold else None
                    # Contact changes are TEST conditions, never used to tune masses/forces.
                    ok=bool(comparison and comparison['stable'] and comparison['min_speed_ratio']>=1.05
                            and comparison['max_work_per_distance_ratio']<=1.10)
                    holdouts.append(dict(factor=factor,passed=ok,comparison=comparison));confirmed &= ok
                record['holdouts']=holdouts
                if confirmed:
                    # Longer independent run; require the actual mechanics to keep working.
                    long=launch(run,f'long_{i}',c['params'],1 if a.quick_test else 6,deadline)
                    record['long_run_passed']=bool(long and long['passed'])
                    if long and long['passed']:
                        winners.append((cmp['min_speed_ratio'],dict(label='QUICK_TEST_ONLY' if a.quick_test else 'SIMULATION_IMPROVEMENT_VALIDATED',
                            run_dir=result['run_dir'],params=c['params'],comparison=cmp,
                            long_run_dir=long['run_dir'],holdouts=holdouts)))
            summarize(run,'VALIDATING',screen,finals,selected)
        if winners:
            selected=max(winners,key=lambda x:x[0])[1]
            status='QUICK_TEST_COMPLETE' if a.quick_test else 'IMPROVED_CRAWL_SAVED'
        elif a.quick_test:status='QUICK_TEST_COMPLETE'
        else:status='NO_CONFIRMED_IMPROVEMENT_BASELINE_PRESERVED'
        if sha(a.source)!=read(run/'manifest.json')['source_sha256']:
            status='ORIGINAL_SOURCE_CHANGED_EXTERNALLY';selected={}
    except KeyboardInterrupt:
        status='STOPPED_BY_USER_EARLIER_RESULTS_SAVED'
    except Exception:
        status='ERROR_NO_SUCCESS_CLAIM';(run/'error.txt').write_text(traceback.format_exc(),encoding='utf-8')
        traceback.print_exc()
    finally:
        summarize(run,status,screen,finals,selected)
        if awake:ctypes.windll.kernel32.SetThreadExecutionState(0x80000000)
        lock.close()
        print('CRAWL NIGHT FINISHED:',status,flush=True)
        print('REPORT:',run/'SUMMARY.md',flush=True)
        print('SHARE:',run/'results_bundle.zip',flush=True)
        print('VIEW: .venv\\Scripts\\python.exe crawl_night.py --view-best --direction forward',flush=True)


if __name__=='__main__': main()
