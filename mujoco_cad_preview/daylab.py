"""Local time-budgeted quadruped lab. No cloud jobs and no hardware control.

Run: .venv\\Scripts\\python.exe daylab.py --hours 12
View latest best candidate: ... daylab.py --view-best --direction forward
Four configurations x two seeds, then bounded continuation of promising ones.
An error, timeout, or no promising candidate can end the batch before 12 hours.
"""
from __future__ import annotations
import argparse
import csv
import ctypes
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import traceback
import zipfile

ROOT=Path(__file__).resolve().parent


def write_json(path, value):
    path=Path(path); path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_suffix(path.suffix+'.tmp')
    temp.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    temp.replace(path)


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def atomic_policy(model,path):
    path=Path(path)
    tmp=path.with_name(path.stem+'.tmp.zip')
    model.save(str(tmp)); tmp.replace(path)


def common_args():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--hours',type=float,default=12.)
    p.add_argument('--smoke',action='store_true',help='Tiny training tests, not walking training.')
    p.add_argument('--view-best',action='store_true')
    p.add_argument('--direction',choices=['forward','backward','stop'],default='forward')
    p.add_argument('--worker',choices=['preflight','train'])
    p.add_argument('--run-dir',type=Path)
    p.add_argument('--out',type=Path)
    p.add_argument('--variant',default='C_posture')
    p.add_argument('--seed',type=int,default=11)
    p.add_argument('--seconds',type=float,default=2400.)
    p.add_argument('--load',type=Path)
    return p


def preflight(run,smoke):
    import numpy as np
    import mujoco
    from stable_baselines3.common.env_checker import check_env
    from daylab_env import DayEnv,calibrate_gait,evaluate
    source=run/'source.xml'
    env=DayEnv(source,'A_reference')
    if env.rest_height < .075 or env.rest_upright < .94:
        raise RuntimeError('Zero-target gravity standing is not stable enough: stop before PPO.')
    check_env(env,warn=True,skip_render_check=True)
    obs,_=env.reset(seed=501)
    for _ in range(150):
        obs,_,term,_,info=env.step(np.zeros(12))
        if term:
            raise RuntimeError('Zero-target baseline fell. No long training will start.')
    baseline={'height_m':info['height_m'],'tilt_deg':info['tilt_deg'],
              'foot_contacts':sum(info['contacts']),'loop_gap_m':info['loop_gap_m']}
    if sum(info['contacts'])<3:
        raise RuntimeError('Fewer than three baseline foot contacts; inspect geometry first.')
    env.close()
    gait=calibrate_gait(source)
    write_json(run/'gait.json',gait)
    # Parameter sweep under FREE BASE + GRAVITY + CONTACT, not an animation.
    grid=[(.012,.012,2.4),(.016,.020,2.4),(.012,.020,3.2)]
    if smoke:
        grid=grid[:1]
    references=[]
    for amplitude,lift,period in grid:
        candidate=dict(gait,amplitude=amplitude,lift=lift,period=period)
        result=evaluate(source,'D_gait_residual',candidate,policy=None,
                        commands=('forward','backward'),seeds=(501,),seconds=6.)
        references.append({'amplitude':amplitude,'lift':lift,'period':period,'evaluation':result})
        print('REFERENCE',amplitude,lift,period,'score',round(result['score'],3),flush=True)
    choice=max(references,key=lambda x:x['evaluation']['score'])
    gait.update({k:choice[k] for k in ('amplitude','lift','period')})
    write_json(run/'gait.json',gait)
    zero=evaluate(source,'A_reference',gait,commands=('forward','backward','stop'),seeds=(501,),seconds=6.)
    result={'status':'PASS','simulation_only':True,'mujoco_version':mujoco.__version__,
            'baseline':baseline,'zero_control':zero,'procedural_gait_sweep':references,
            'warning':'PASS means runnable physics, NOT verified motor mapping or walking.'}
    write_json(run/'preflight.json',result)
    print('PREFLIGHT PASS: gravity, ground contacts and finite simulation verified.',flush=True)


def train(args):
    import math
    import numpy as np
    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import BaseCallback
    from stable_baselines3.common.monitor import Monitor
    from daylab_env import CONFIGS,DayEnv,evaluate
    out=args.out; out.mkdir(parents=True,exist_ok=True)
    source=args.run_dir/'source.xml'
    gait=read_json(args.run_dir/'gait.json')
    config=CONFIGS[args.variant]
    torch.set_num_threads(2)
    deadline=time.monotonic()+args.seconds
    class BudgetCallback(BaseCallback):
        def _on_rollout_start(self):
            if config['bounded_std']:
                with torch.no_grad():
                    self.model.policy.log_std.clamp_(math.log(.15),math.log(.5))
        def _on_step(self):
            return time.monotonic()<deadline
    env=Monitor(DayEnv(source,args.variant,gait,randomize=False))
    if args.load:
        model=PPO.load(str(args.load),env=env,device='cpu')
    else:
        model=PPO('MlpPolicy',env,seed=args.seed,device='cpu',verbose=0,
            n_steps=64 if args.smoke else 512,batch_size=32 if args.smoke else 128,
            n_epochs=2 if args.smoke else 5,learning_rate=3e-4,gamma=.99,gae_lambda=.95,
            clip_range=.2,target_kl=.03,ent_coef=0. if config['bounded_std'] else .005,
            policy_kwargs={'net_arch':[64,64], 'log_std_init':-1.2 if config['bounded_std'] else 0.})
    metadata={'variant':args.variant,'seed':args.seed,'new_observation_dim':59,
        'source_sha256':sha(source),'gait':gait,'config':config,'load':str(args.load) if args.load else None,
        'old_models_modified':False,'hardware_control':False}
    write_json(out/'config.json',metadata)
    atomic_policy(model,out/'last.zip')
    start_steps=int(model.num_timesteps)
    best=-float('inf'); history=[]; flat=0; status='TIME_BUDGET'
    try:
        while time.monotonic()<deadline:
            model.learn(total_timesteps=128 if args.smoke else 25088,
                        callback=BudgetCallback(),reset_num_timesteps=False,progress_bar=False)
            atomic_policy(model,out/'last.zip')
            result=evaluate(source,args.variant,gait,policy=model,
                            commands=('forward','backward','stop'),seeds=(501,) if args.smoke else (501,502),seconds=6.)
            result['steps']=int(model.num_timesteps)
            result['policy_std_mean']=float(torch.exp(model.policy.log_std).mean().detach().cpu())
            history.append(result)
            write_json(out/'last_evaluation.json',result)
            if result['score']>best+.01:
                best=result['score']; flat=0
                atomic_policy(model,out/'best.zip')
                write_json(out/'best_evaluation.json',result)
            else:
                flat+=1
            write_json(out/'history.json',history)
            print(f"{args.variant} seed={args.seed} steps={model.num_timesteps} "
                  f"score={result['score']:.3f} tail_ratio={result['tail_ratio']:.2f} "
                  f"falls={result['fall_rate']:.0%} candidate={result['candidate_rate']:.0%}",flush=True)
            if args.smoke:
                status='SMOKE_COMPLETE'; break
            # At least ~300k new decisions; never equate reward plateau with skill.
            if model.num_timesteps-start_steps>=300000 and flat>=8 and result['candidate_rate']==0:
                status='NO_IMPROVEMENT_STOP'; break
        if (out/'best.zip').exists():
            best_model=PPO.load(str(out/'best.zip'),device='cpu')
            # Held-out seeds/contact variations, NOT used for model selection.
            hold=evaluate(source,args.variant,gait,policy=best_model,
                          commands=('forward','backward','stop'),
                          seeds=(901,) if args.smoke else (901,902,903),seconds=6.,randomize=True)
            write_json(out/'holdout.json',hold)
    except KeyboardInterrupt:
        status='INTERRUPTED'
    finally:
        atomic_policy(model,out/'last.zip')
        write_json(out/'worker_status.json',{'status':status,'steps':int(model.num_timesteps),
            'new_steps':int(model.num_timesteps)-start_steps,'simulation_only':True})
        env.close()


def launch(run,kind,out,seconds,smoke=False,variant=None,seed=11,load=None):
    out.mkdir(parents=True,exist_ok=True)
    cmd=[sys.executable,'-u',str(Path(__file__).resolve()),'--worker',kind,
         '--run-dir',str(run),'--out',str(out),'--seconds',str(seconds),'--seed',str(seed)]
    if smoke: cmd.append('--smoke')
    if variant: cmd+=['--variant',variant]
    if load: cmd+=['--load',str(load)]
    log=out/'console.log'
    with log.open('w',encoding='utf-8',buffering=1) as fh:
        process=subprocess.Popen(cmd,cwd=ROOT,stdout=fh,stderr=subprocess.STDOUT)
        end=time.monotonic()+seconds+60
        try:
            while process.poll() is None:
                if time.monotonic()>end:
                    process.terminate()
                    try: process.wait(timeout=10)
                    except subprocess.TimeoutExpired: process.kill(); process.wait()
                    write_json(out/'timeout.json',{'status':'WATCHDOG_TIMEOUT','seconds':seconds})
                    return False
                time.sleep(.5)
        except KeyboardInterrupt:
            # Best/last periodic checkpoints already exist; terminate only this child.
            process.terminate()
            try: process.wait(timeout=10)
            except subprocess.TimeoutExpired: process.kill(); process.wait()
            raise
    if process.returncode:
        print(log.read_text(encoding='utf-8')[-4500:],flush=True)
    return process.returncode==0


def collect(run):
    rows=[]
    for p in sorted(run.glob('*/best_evaluation.json')):
        cfg=read_json(p.parent/'config.json'); score=read_json(p)
        hold=read_json(p.parent/'holdout.json') if (p.parent/'holdout.json').exists() else {}
        rows.append({'trial':p.parent.name,'variant':cfg['variant'],'seed':cfg['seed'],
                     'score':score['score'],'candidate_rate':score['candidate_rate'],
                     'tail_ratio':score['tail_ratio'],'fall_rate':score['fall_rate'],
                     'holdout_candidate_rate':hold.get('candidate_rate'),
                     'best_policy':str(p.parent/'best.zip')})
    return sorted(rows,key=lambda x:x['score'],reverse=True)


def summarize(run,status):
    rows=collect(run)
    write_json(run/'ranking.json',rows)
    if rows:
        with (run/'ranking.csv').open('w',newline='',encoding='utf-8-sig') as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    summary=['# BGH DayLab result', '', 'Status: '+status,
             'SIMULATION ONLY. A highest score is not a walking success.',
             'No original CAD, motor limits, old PPO weights or real robot were changed.',
             '', '| trial | tail speed ratio | candidate rate | holdout candidate | score |',
             '|---|---:|---:|---:|---:|']
    for r in rows:
        summary.append(f"| {r['trial']} | {r['tail_ratio']:.3f} | {r['candidate_rate']:.2f} | {r['holdout_candidate_rate']} | {r['score']:.3f} |")
    summary+=['','Read preflight.json and per-trial holdout.json before calling this walking.',
              'Tail metrics use the last 2 seconds; settling/crouching displacement alone is not success.',
              'Inspect toe trajectories and contact timing visually as well.',
              'The candidate heuristic is NOT a real-hardware safety certification.']
    (run/'SUMMARY.md').write_text('\n'.join(summary),encoding='utf-8')
    write_json(run/'status.json',{'status':status,'trials_with_checkpoints':len(rows),'time':time.time()})
    with zipfile.ZipFile(run/'results_bundle.zip','w',compression=zipfile.ZIP_DEFLATED) as z:
        for p in run.rglob('*'):
            if p.is_file() and p.suffix in ('.json','.csv','.md','.log') and p.stat().st_size<8_000_000:
                z.write(p,p.relative_to(run))
    return rows


def batch(args):
    if not .05<=args.hours<=12:
        raise ValueError('--hours must be 0.05..12')
    from daylab_env import CONFIGS
    source=next((ROOT/'output'/n for n in ('physics_rl_v0.xml','physics_v1.xml')
                 if (ROOT/'output'/n).is_file()),None)
    if source is None:
        raise FileNotFoundError('Existing output/physics_v1.xml or physics_rl_v0.xml required.')
    top=ROOT/'output/daylab';top.mkdir(parents=True,exist_ok=True)
    run=top/time.strftime('%Y%m%d_%H%M%S')
    run.mkdir(exist_ok=False)
    (top/'LATEST.txt').write_text(str(run),encoding='utf-8')
    shutil.copy2(source,run/'source.xml')
    versions={n:importlib.metadata.version(n) for n in ('mujoco','numpy','gymnasium','stable-baselines3','torch')}
    # Budget includes preflight and summaries. Training cutoff precedes it.
    total=1200. if args.smoke else args.hours*3600
    deadline=time.monotonic()+total
    manifest={'source':str(source),'source_sha256':sha(source),'versions':versions,
              'max_wall_seconds':total,'configurations':CONFIGS,
              'seeds':[11] if args.smoke else [11,29],
              'scope':'forward/back/stop only, no untrained lateral/turn claims'}
    write_json(run/'manifest.json',manifest)
    print('LOCAL DAYLAB STARTED:',run,flush=True)
    print('No GUI during training. Check per-trial console.log for live PPO metrics.',flush=True)
    awake=False
    if os.name=='nt':
        try:
            awake=bool(ctypes.windll.kernel32.SetThreadExecutionState(0x80000001))
            print('Windows automatic sleep inhibition:',awake,flush=True)
        except Exception: pass
    status='COMPLETED'
    try:
        pre_budget=min(300.,max(30.,total*.08))
        if not launch(run,'preflight',run/'preflight_logs',pre_budget,args.smoke):
            status='PREFLIGHT_FAILED_NO_TRAINING'; return
        print('PREFLIGHT PASSED. Beginning independent training trials.',flush=True)
        seeds=(11,) if args.smoke else (11,29)
        trials=[(variant,seed) for seed in seeds for variant in CONFIGS]
        slot=45. if args.smoke else min(2400.,total*.48/len(trials))
        failures=0
        for variant,seed in trials:
            remaining=deadline-time.monotonic()-120
            if remaining<30: status='TIME_BUDGET';break
            name=f'{variant}_seed{seed}'
            print('START TRIAL:',name,flush=True)
            good=launch(run,'train',run/name,min(slot,remaining),args.smoke,variant,seed)
            failures=0 if good else failures+1
            summarize(run,'RUNNING')
            if failures>=3:
                status='THREE_CONSECUTIVE_ERRORS';break
        if not args.smoke and status not in ('THREE_CONSECUTIVE_ERRORS','TIME_BUDGET'):
            rows=collect(run)
            promising=[r for r in rows if r['tail_ratio']>=.15 and r['fall_rate']<=.25][:2]
            if not promising:
                status='NO_PROMISING_POLICY_STOPPED_EARLY'
            else:
                for k,r in enumerate(promising):
                    remaining=deadline-time.monotonic()-120
                    if remaining<60: break
                    budget=min(3*3600.,remaining/(len(promising)-k))
                    print('CONTINUE PROMISING CANDIDATE:',r['trial'],flush=True)
                    launch(run,'train',run/('continue_'+r['trial']),budget,False,
                           r['variant'],r['seed'],Path(r['best_policy']))
                    summarize(run,'RUNNING_CONTINUATION')
        if sha(source)!=manifest['source_sha256']:
            status+='; SOURCE_CHANGED_EXTERNALLY'
    except KeyboardInterrupt:
        status='USER_INTERRUPTED'
    except Exception:
        status='BATCH_EXCEPTION'
        (run/'error.log').write_text(traceback.format_exc(),encoding='utf-8')
        traceback.print_exc()
    finally:
        summarize(run,status)
        if awake:
            ctypes.windll.kernel32.SetThreadExecutionState(0x80000000)
        print('DAYLAB FINISHED:',status,flush=True)
        print('REPORT:',run/'SUMMARY.md',flush=True)
        print('SHARE:',run/'results_bundle.zip',flush=True)
        print('VIEW: .venv\\Scripts\\python.exe daylab.py --view-best --direction forward',flush=True)


def view(args):
    import numpy as np
    import mujoco.viewer
    from stable_baselines3 import PPO
    from daylab_env import DayEnv
    run=Path((ROOT/'output/daylab/LATEST.txt').read_text(encoding='utf-8').strip())
    ranking=read_json(run/'ranking.json')
    if not ranking:
        raise RuntimeError('No candidate saved; read SUMMARY.md and error logs.')
    row=ranking[0]; cfg=read_json(Path(row['best_policy']).parent/'config.json')
    policy=PPO.load(row['best_policy'],device='cpu')
    env=DayEnv(run/'source.xml',row['variant'],cfg['gait'],command=args.direction,
               randomize=False,visuals=True,episode_seconds=8.)
    print('VIEWING HIGHEST-RANKED CANDIDATE, NOT GUARANTEED WALKING:',row,flush=True)
    obs,_=env.reset(seed=901)
    with mujoco.viewer.launch_passive(env.model,env.data) as viewer:
        viewer.cam.distance=.7;viewer.cam.azimuth=125;viewer.cam.elevation=-20
        while viewer.is_running():
            start=time.perf_counter()
            action,_=policy.predict(obs,deterministic=True)
            with viewer.lock():
                obs,_,term,trunc,info=env.step(action)
                viewer.cam.lookat[:]=env.data.xpos[env.base]
            viewer.sync()
            time.sleep(max(0.,env.dt-(time.perf_counter()-start)))
            if term or trunc:
                print('Episode end:',info['fall'],'height:',info['height_m'],flush=True)
                with viewer.lock(): obs,_=env.reset(seed=901)
    env.close()


def main():
    args=common_args().parse_args()
    if args.worker=='preflight': preflight(args.run_dir,args.smoke)
    elif args.worker=='train': train(args)
    elif args.view_best: view(args)
    else: batch(args)


if __name__=='__main__':
    try: main()
    except Exception:
        traceback.print_exc()
        sys.exit(1)
