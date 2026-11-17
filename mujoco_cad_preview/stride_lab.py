"""All-direction speed AND smoothness optimization. Not PPO. Local PC only.
Start: --start --hours 24 --workers 0 --trials 1200
Status/stop/view: --status, --stop, --view backward
Completion: local Windows sound and persistent popup, after finalization.
"""
from __future__ import annotations
import os
for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS'):os.environ[k]='1'
import argparse,csv,ctypes,hashlib,json,random,shutil,subprocess,sys,time,traceback,zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parent
FILES=('linkage_core.py','omni_trial.py','omni_parallel.py','flow_trial.py','flow_notify.py','stride_trial.py','stride_lab.py')

def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def write(p,x):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True);tmp=p.with_suffix(p.suffix+'.tmp')
    tmp.write_text(json.dumps(x,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf-8');tmp.replace(p)
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def compare(a,b,min_speed=5.,ratio=2.):
    if not a.get('passed') or not b.get('passed') or not a.get('metrics') or not b.get('metrics'):return None
    if (a['direction'],a['cycles'],a['friction'])!=(b['direction'],b['cycles'],b['friction']):raise ValueError('Unlike tests')
    x,y=a['metrics'],b['metrics'];speed=x['speed']/max(y['speed'],1e-9);rot=a['direction'].startswith('turn')
    target=max(ratio*y['speed'],.15 if rot else min_speed)
    work=x['work_per_progress']/max(y['work_per_progress'],1e-9)
    # Never loosen acceleration just because the policy is faster. Both raw
    # 50Hz acceleration and jerk must remain within narrow baseline tolerance.
    checks=dict(speed_2x_and_absolute=x['speed']>=target,
        sway_reduced=x['sway_mm']<=max(5.,y['sway_mm']*.90),
        acceleration_not_worse=x['accel_rms']<=max(.01,y['accel_rms']*1.05),
        jerk_not_worse=x['jerk_rms']<=max(.1,y['jerk_rms']*1.10),
        fewer_reversals=x['reverse_fraction']<=min(.15,y['reverse_fraction']+.01),
        few_stops=x['stall_fraction']<=.08 and x['longest_stall_s']<=.20,
        posture=x['max_tilt_deg']<=min(8.,y['max_tilt_deg']+1.),
        work_not_worse=work<=1.05)
    score=speed-.7*max(0,x['accel_rms']/max(y['accel_rms'],.01)-1.)
    score-=.7*max(0,x['sway_mm']/max(y['sway_mm'],5.)-1.)+2*x['reverse_fraction']+x['stall_fraction']
    return dict(target_met=all(checks.values()),checks=checks,target_speed=target,speed_unit=x['speed_unit'],
        speed_ratio=speed,stretch_3x=speed>=3.,work_ratio=work,sway_ratio=x['sway_mm']/max(y['sway_mm'],1.),
        acceleration_ratio=x['accel_rms']/max(y['accel_rms'],1e-9),jerk_ratio=x['jerk_rms']/max(y['jerk_rms'],1e-9),score=float(score))

class Queue:
    def __init__(self,run,workers,deadline):self.run=run;self.workers=workers;self.deadline=deadline;self.done=0;self.total=0
    def event(self,event,**x):
        with (self.run/'events.jsonl').open('a',encoding='utf-8') as f:f.write(json.dumps(dict(event=event,time=time.time(),**x))+'\n')
    def jobs(self,specs,phase,until=None):
        todo=[dict(s) for s in specs];active=[];results={};self.total+=len(todo);heartbeat=0.
        end=min(until or self.deadline,self.deadline)
        def status():write(self.run/'status.json',dict(state='RUNNING',phase=phase,pid=os.getpid(),updated=time.time(),
            workers=self.workers,finished_jobs=self.done,submitted_jobs=self.total,
            active=[dict(id=j['id'],pid=j['p'].pid,elapsed_s=round(time.monotonic()-j['start'],1)) for j in active]))
        try:
            while todo or active:
                if (self.run/'STOP').exists():raise KeyboardInterrupt('STOP requested')
                if time.monotonic()>end:break
                while todo and len(active)<self.workers:
                    s=todo.pop(0);ident=s.pop('id');out=self.run/ident;out.mkdir(parents=True,exist_ok=True)
                    if (out/'report.json').exists():results[ident]=read(out/'report.json');self.done+=1;continue
                    write(out/'task.json',dict(s,source=str(self.run/'source.xml'),out=str(out)))
                    log=(out/'console.log').open('w',encoding='utf-8')
                    proc=subprocess.Popen([sys.executable,'-u',str(self.run/'code/stride_trial.py'),str(out/'task.json')],
                        cwd=self.run/'code',stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,
                        creationflags=subprocess.CREATE_NO_WINDOW if os.name=='nt' else 0)
                    active.append(dict(id=ident,p=proc,log=log,out=out,start=time.monotonic()))
                    self.event('start',id=ident,pid=proc.pid);print('START',ident,proc.pid,flush=True)
                for j in active[:]:
                    timeout=time.monotonic()-j['start']>2400
                    if timeout and j['p'].poll() is None:j['p'].terminate()
                    if j['p'].poll() is None and not timeout:continue
                    try:j['p'].wait(timeout=5)
                    except subprocess.TimeoutExpired:j['p'].kill();j['p'].wait()
                    j['log'].close();active.remove(j);self.done+=1
                    r=read(j['out']/'report.json') if (j['out']/'report.json').exists() else dict(passed=False,metrics=None,failure='WORKER_TIMEOUT_OR_ERROR')
                    write(j['out']/'report.json',r);results[j['id']]=r;self.event('end',id=j['id'],pid=j['p'].pid,passed=r['passed'])
                    print('END',j['id'],r['passed'],flush=True)
                if time.monotonic()-heartbeat>3:status();heartbeat=time.monotonic()
                time.sleep(.2)
        finally:
            for j in active:
                if j['p'].poll() is None:
                    j['p'].terminate()
                    try:j['p'].wait(timeout=5)
                    except subprocess.TimeoutExpired:j['p'].kill();j['p'].wait()
                j['log'].close();self.event('end',id=j['id'],pid=j['p'].pid,passed=False,interrupted=True)
                write(j['out']/'interrupted.json',dict(reason='TIME_BUDGET_OR_STOP'))
            active=[];status()
        return results

def save_results(run,state,selections,search):
    write(run/'selection.json',selections);write(run/'search.json',search)
    old=read(run/'status.json') if (run/'status.json').exists() else {}
    write(run/'status.json',dict(old,state=state,updated=time.time(),active=[]))
    with (run/'ranking.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.writer(f);w.writerow(['trial','direction','physical_pass','target_met','speed_ratio','sway_ratio','acceleration_ratio'])
        for x in search:
            c=x.get('comparison') or {};w.writerow([x['id'],x['direction'],x['passed'],c.get('target_met'),c.get('speed_ratio'),c.get('sway_ratio'),c.get('acceleration_ratio')])
    lines=['# Speed + smoothness / all directions','Status: '+state,
        'Direct controller optimization, NOT PPO. Target is >=2x latest matched baseline, NOT a guarantee.',
        'Body dynamics, masses, actuator limits and ground contact remain enabled and unchanged.',
        'TARGET_MET is distinct from PHYSICS_ONLY and PREVIOUS_RESULT. 1x recorded simulation replay only.','']
    for d,s in selections.items():lines.append(d+': '+s['label'])
    (run/'SUMMARY.md').write_text('\n'.join(lines),encoding='utf-8')
    with zipfile.ZipFile(run/'results_bundle.tmp.zip','w',zipfile.ZIP_DEFLATED) as z:
        for p in run.rglob('*'):
            if p.is_file() and (p.suffix in ('.json','.jsonl','.md','.csv','.py','.xml') or p.name=='error.txt'):z.write(p,str(p.relative_to(run)))
    (run/'results_bundle.tmp.zip').replace(run/'results_bundle.zip')
    n=sum(x['label']=='TARGET_2X_CONFIRMED' for x in selections.values())
    write(run/'completion.json',dict(state=state,target_met=n,directions=len(selections),finished_at=time.time(),bundle=str(run/'results_bundle.zip')))

def reference_catalog(root,source,dirs):
    from omni_trial import DEFAULT as OLD
    ref={d:dict(kind='omni',params=dict(OLD,stride_mm=16. if d in ('forward','backward') else 6.,yaw_deg=1.),origin='conservative fallback') for d in dirs}
    h=sha(source)
    for folder,kind in [('omni_parallel','omni'),('flow_lab','flow')]:
        p=root/'output'/folder/'LATEST.txt'
        if not p.exists():continue
        try:
            run=Path(p.read_text('utf-8').strip());m=read(run/'manifest.json')
            if m.get('source_sha256')!=h:continue
            for d,s in read(run/'selection.json').items():
                if d in ref and s.get('run_dir') and s.get('params'):
                    r=read(Path(s['run_dir'])/'report.json')
                    if r.get('passed'):ref[d]=dict(kind=kind,params=s['params'],origin=str(run))
        except (OSError,ValueError,KeyError):continue
    return ref

def optimize(run,args):
    from omni_parallel import lock_file
    from stride_trial import DEFAULT,BOUNDS
    man=read(run/'manifest.json');dirs=man['directions'];refs=man['references'];workers=man['workers']
    start=time.monotonic();deadline=start+args.hours*3600;batch=Queue(run,workers,deadline)
    rng=random.Random(args.seed);search=[];selected={d:dict(label='NO_VALIDATED_RESULT',run_dir=None) for d in dirs}
    state='NO_TARGET_MET';held=[];awake=False
    spec=lambda name,d,p,n,kind='new',mu=1.:dict(id=name,direction=d,params=p,cycles=n,kind=kind,friction=mu)
    try:
        held.append(lock_file(run.parent/'RUNNING.lock'))
        for name in ('crawl_night','omni_parallel','flow_lab'):
            p=Path(man['root'])/'output'/name/'RUNNING.lock'
            if p.exists():held.append(lock_file(p))
        if os.name=='nt':awake=bool(ctypes.windll.kernel32.SetThreadExecutionState(0x80000001))
        n=1 if args.quick_test else 3
        base=batch.jobs([spec('reference_'+d,d,refs[d]['params'],n,refs[d]['kind']) for d in dirs],'MATCH_CURRENT_REFERENCES')
        # Direction coverage is never silently dropped because one new seed fails.
        if not any(r.get('passed') for r in base.values()):state='SOURCE_BASELINES_FAILED';return
        presets=[dict(DEFAULT),dict(DEFAULT,height_mm=108.,stride_mm=28.,slot_s=.85,velocity_ff=1.),
            dict(DEFAULT,height_mm=108.,stride_mm=32.,slot_s=.7,velocity_ff=1.,path_weight=.4),
            dict(DEFAULT,stride_mm=24.,slot_s=1.5,height_mm=115.,velocity_ff=.4),
            dict(DEFAULT,stride_mm=16.,slot_s=1.7,height_mm=115.,velocity_ff=.4),
            dict(DEFAULT,stride_mm=10.,slot_s=1.5,height_mm=115.,velocity_ff=.6),
            dict(DEFAULT,stride_mm=18.,slot_s=1.1,order=0),dict(DEFAULT,stride_mm=22.,slot_s=.85,order=5)]
        elites={d:[] for d in dirs};seen=set();budget=deadline-min(args.hours*3600*.4,8*3600)
        count=min(args.trials,2) if args.quick_test else args.trials
        for offset in range(0,count,workers*2):
            if not args.quick_test and time.monotonic()>budget:break
            jobs=[];lookup={}
            for i in range(offset,min(count,offset+workers*2)):
                d=dirs[i%len(dirs)];roundno=i//len(dirs)
                if roundno<len(presets):p=dict(presets[roundno])
                else:
                    p=dict(rng.choice(elites[d][:6])[1]) if elites[d] and rng.random()<.8 else dict(rng.choice(presets))
                    keys=[k for k in DEFAULT if k!='yaw_deg' or d.startswith('turn')]
                    for key in rng.sample(keys,3):
                        lo,hi=BOUNDS[key]
                        p[key]=rng.randrange(6) if key=='order' else round(max(lo,min(hi,p[key]+rng.gauss(0,(hi-lo)*.13))),4)
                key=(d,tuple(p.values()))
                if key in seen:p['path_weight']=round(rng.uniform(.10,.60),5)
                seen.add(key);name=f'trial_{i:04d}_{d}';lookup[name]=(d,p);jobs.append(spec(name,d,p,n))
            results=batch.jobs(jobs,'SEARCH_ALL_DIRECTIONS',deadline if args.quick_test else budget)
            for name,r in results.items():
                d,p=lookup[name];c=compare(r,base.get('reference_'+d,{}),args.min_mm_s,args.ratio)
                row=dict(id=name,direction=d,params=p,passed=r['passed'],comparison=c);search.append(row)
                if r['passed']:
                    score=c['score'] if c else (r.get('metrics') or {}).get('speed',0.)
                    elites[d].append((score,p));elites[d].sort(key=lambda x:x[0],reverse=True)
            write(run/'search.json',search)
        if args.quick_test:state='SMOKE_ONLY_NOT_QUALIFIED';return
        candidates={}
        for d in dirs:
            feasible=[x for x in search if x['direction']==d and x['passed']]
            feasible.sort(key=lambda x:((x.get('comparison') or {}).get('target_met',False),(x.get('comparison') or {}).get('score',0.)),reverse=True)
            unique=[];keys=set()
            for x in feasible:
                k=tuple(sorted(x['params'].items()))
                if k not in keys:unique.append(x);keys.add(k)
            for k,x in enumerate(unique[:3]):candidates['confirm_'+d+'_'+str(k)]=(d,x['params'])
        specs=[spec(name,d,p,24) for name,(d,p) in candidates.items()]
        specs +=[spec('confirm_reference_'+d,d,refs[d]['params'],24,refs[d]['kind']) for d in dirs]
        checks=batch.jobs(specs,'24_CYCLE_NOMINAL_VALIDATION')
        hold_jobs=[]
        for name,r in checks.items():
            if not r['passed']:continue
            for mu in (.9,1.1):hold_jobs.append(spec(name+'_mu'+str(mu),r['direction'],r['params'],12,r['kind'],mu))
        holds=batch.jobs(hold_jobs,'12_CYCLE_FRICTION_HOLDOUTS')
        # Equal-cycle reference measurements for every perturbed condition.
        for name,r in checks.items():
            if not r['passed'] or name.startswith('confirm_reference_'):continue
            if not all(holds.get(name+'_mu'+str(mu),{}).get('passed') for mu in (.9,1.1)):continue
            d=r['direction'];refname='confirm_reference_'+d;nom=compare(r,checks.get(refname,{}),args.min_mm_s,args.ratio)
            hc=[compare(holds[name+'_mu'+str(mu)],holds.get(refname+'_mu'+str(mu),{}),args.min_mm_s,args.ratio) for mu in (.9,1.1)]
            target=bool(nom and nom['target_met'] and all(c and c['target_met'] for c in hc))
            candidate=dict(label='TARGET_2X_CONFIRMED' if target else 'PHYSICS_ONLY_NOT_SPEED_SMOOTHNESS_TARGET',
                run_dir=str(run/name),params=r['params'],comparison=nom,holdout_comparisons=hc,
                target_met=target,kind='new',speed=r['metrics']['speed'],metrics=r['metrics'])
            old=selected[d]
            if not old.get('run_dir') or (target,nom['score'] if nom else 0.)>(old.get('target_met',False),(old.get('comparison') or {}).get('score',0.)):
                selected[d]=candidate
        for d in dirs:
            name='confirm_reference_'+d;r=checks.get(name,{})
            if not selected[d].get('run_dir') and r.get('passed'):
                selected[d]=dict(label='PREVIOUS_RESULT_ONLY',run_dir=str(run/name),params=r['params'],kind=r['kind'],target_met=False)
        # Test all available new motion directions in one state; changes occur
        # at full-cycle boundaries. Never claim arbitrary joystick switching.
        available=[d for d in dirs if selected[d].get('kind')=='new' and selected[d].get('run_dir')]
        if len(available)>=2:
            params={d:selected[d]['params'] for d in available};chain=available[:];chain.insert(len(chain)//2,'stop');chain.append('stop')
            t=spec('sequence',available[0],params[available[0]],1);t.update(commands=chain,command_params=params)
            rr=batch.jobs([t],'NO_RESET_DIRECTION_SEQUENCE').get('sequence',{})
            selected['sequence']=dict(label='SEQUENCE_PHYSICS_PASS' if rr.get('passed') else 'SEQUENCE_FAILED',run_dir=str(run/'sequence') if rr.get('passed') else None,commands=chain)
        targets=sum(selected[d].get('target_met',False) for d in dirs)
        state='ALL_DIRECTION_TARGETS_MET' if targets==len(dirs) else 'PARTIAL_TARGETS_MET' if targets else 'NO_2X_SMOOTHNESS_TARGET_MET'
    except KeyboardInterrupt:state='STOPPED_RESULTS_PRESERVED'
    except Exception:
        state='BATCH_ERROR';(run/'error.txt').write_text(traceback.format_exc(),encoding='utf-8');traceback.print_exc()
    finally:
        if sha(run/'source.xml')!=man['source_sha256']:state='SOURCE_CHANGED_NO_RESULT';selected={}
        save_results(run,state,selected,search)
        for f in held:f.close()
        if awake:ctypes.windll.kernel32.SetThreadExecutionState(0x80000000)
        print('FINISHED',state,run/'results_bundle.zip',flush=True)

def supervisor(run,args):
    cmd=[sys.executable,'-u',str(run/'code/stride_lab.py'),'--master','--run',str(run),'--hours',str(args.hours),
         '--trials',str(args.trials),'--seed',str(args.seed),'--ratio',str(args.ratio),'--min-mm-s',str(args.min_mm_s)]
    if args.quick_test:cmd+=['--quick-test']
    with (run/'master.log').open('w',encoding='utf-8') as log:
        kw=dict(cwd=run/'code',stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT)
        if os.name=='nt':kw['creationflags']=subprocess.CREATE_NO_WINDOW|subprocess.CREATE_NEW_PROCESS_GROUP
        else:kw['start_new_session']=True
        proc=subprocess.Popen(cmd,**kw);write(run/'master_pid.json',{'pid':proc.pid});deadline=time.monotonic()+args.hours*3600+180
        while proc.poll() is None:
            if time.monotonic()>deadline:
                (run/'STOP').write_text('supervisor time limit')
                try:proc.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    if os.name=='nt':subprocess.run(['taskkill','/PID',str(proc.pid),'/T','/F'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=15)
                    else:
                        import signal;os.killpg(proc.pid,signal.SIGTERM)
                    proc.wait(timeout=10)
                break
            time.sleep(.5)
    if not (run/'completion.json').exists():
        write(run/'completion.json',dict(state='MASTER_ERROR',returncode=proc.returncode,finished_at=time.time()))
        write(run/'status.json',dict(state='MASTER_ERROR',active=[],updated=time.time()))
    from flow_notify import notify
    result=read(run/'completion.json');text=result['state']+'\n2x + smoothness targets: '+str(result.get('target_met',0))
    notify(run,text,silent=args.no_alert)

def replay(run,direction):
    import numpy as np,mujoco,mujoco.viewer
    from xml.etree import ElementTree as ET
    s=read(run/'selection.json').get(direction,{})
    if not s.get('run_dir'):
        print('No qualified or physically confirmed recording for',direction,'. Read SUMMARY.md; old results are intact.');return
    out=Path(s['run_dir']);r=read(out/'report.json')
    if not r['passed']:raise RuntimeError('Recorded physical test failed')
    print('RESULT CATEGORY:',s['label']);print('ACTUAL METRICS:',r.get('metrics'))
    print('1X PHYSICS RECORDING. NOT LIVE AI. PHYSICS_ONLY IS NOT A 2X/SMOOTHNESS SUCCESS.')
    xml=ET.fromstring((run/'source.xml').read_text('utf-8-sig'));source=Path(read(run/'manifest.json')['source_original']);assets={}
    for mesh in xml.findall('asset/mesh'):
        n=mesh.get('file');candidates=[source.parent/'cad_meshes'/n,ROOT/'output/cad_meshes'/n]
        p=next((x for x in candidates if x.is_file()),None)
        if p is None:raise FileNotFoundError('Missing visual mesh '+n)
        assets[n]=p.read_bytes()
    m=mujoco.MjModel.from_xml_string(ET.tostring(xml,encoding='unicode'),assets=assets);data=mujoco.MjData(m)
    with np.load(out/'trajectory.npz',allow_pickle=False) as f:q=f['qpos'].copy();dt=float(f['dt'])
    print('RESULT CATEGORY:',s['label'])
    with mujoco.viewer.launch_passive(m,data) as v:
        v.cam.distance=.9;v.cam.elevation=-20;v.cam.azimuth=145;v.cam.lookat[:]=[0,0,.07];t=time.perf_counter();old=-1
        while v.is_running():
            i=int((time.perf_counter()-t)/dt)%len(q)
            if i!=old:
                with v.lock():data.qpos[:]=q[i];data.qvel[:]=0.;mujoco.mj_forward(m,data)
                v.sync();old=i
            time.sleep(.005)

def main():
    a=argparse.ArgumentParser(description=__doc__)
    a.add_argument('--start',action='store_true');a.add_argument('--hours',type=float,default=24.);a.add_argument('--workers',type=int,default=0)
    a.add_argument('--trials',type=int,default=1200);a.add_argument('--ratio',type=float,default=2.);a.add_argument('--min-mm-s',type=float,default=5.)
    a.add_argument('--seed',type=int,default=20261118);a.add_argument('--commands',default='');a.add_argument('--source',type=Path,default=ROOT/'output/physics_v1.xml')
    a.add_argument('--status',action='store_true');a.add_argument('--stop',action='store_true');a.add_argument('--view');a.add_argument('--list',action='store_true')
    a.add_argument('--test-alert',action='store_true');a.add_argument('--no-alert',action='store_true');a.add_argument('--quick-test',action='store_true')
    a.add_argument('--supervisor',action='store_true');a.add_argument('--master',action='store_true');a.add_argument('--run',type=Path)
    args=a.parse_args();folder=ROOT/'output/stride_lab';folder.mkdir(parents=True,exist_ok=True)
    if args.test_alert:
        from flow_notify import notify;notify(folder/'alert_test','STRIDE_ALERT_TEST',test=True,silent=args.no_alert);return
    if args.supervisor:supervisor(args.run,args);return
    if args.master:optimize(args.run,args);return
    if args.status or args.stop or args.view or args.list:
        run=Path((folder/'LATEST.txt').read_text('utf-8').strip())
        if args.stop:(run/'STOP').write_text('User stop');print('STOP requested');return
        if args.view:replay(run,args.view);return
        print('RUN:',run);print(json.dumps(read(run/('selection.json' if args.list else 'status.json')),ensure_ascii=False,indent=2));return
    if not args.start:a.error('Use --start, --status, --view, --list, --stop or --test-alert')
    if not (.05<=args.hours<=72 and 0<=args.workers<=8 and 1<=args.trials<=5000 and 1.5<=args.ratio<=3 and 0<=args.min_mm_s<=20):a.error('Invalid guarded settings')
    from omni_parallel import resources,lock_file
    from stride_trial import COMMANDS
    dirs=args.commands.split(',') if args.commands else list(COMMANDS)
    if len(set(dirs))!=len(dirs) or any(d not in COMMANDS for d in dirs):a.error('Unknown or duplicate direction')
    if not args.source.is_file():raise FileNotFoundError(str(args.source))
    probe=lock_file(folder/'RUNNING.lock');probe.close()
    for name in ('crawl_night','omni_parallel','flow_lab'):
        p=ROOT/'output'/name/'RUNNING.lock'
        if p.exists():probe=lock_file(p);probe.close()
    for name in FILES:
        if not (ROOT/name).is_file():raise FileNotFoundError(name)
    run=folder/(time.strftime('%Y%m%d_%H%M%S')+f'_{os.getpid()}');run.mkdir();(run/'code').mkdir()
    for name in FILES:shutil.copy2(ROOT/name,run/'code'/name)
    shutil.copy2(args.source,run/'source.xml');info=resources(args.workers)
    write(run/'manifest.json',dict(root=str(ROOT),source_original=str(args.source.resolve()),source_sha256=sha(args.source),
        code_sha256={n:sha(ROOT/n) for n in FILES},workers=info['workers'],resources=info,directions=dirs,
        references=reference_catalog(ROOT,args.source,dirs),ratio_target=args.ratio,min_speed_mm_s=args.min_mm_s,
        hours=args.hours,trials=args.trials,PPO=False,hardware=False,
        versions={n:importlib.metadata.version(n) for n in ('mujoco','numpy','scipy')}))
    write(run/'status.json',dict(state='STARTING',updated=time.time(),workers=info['workers']))
    (folder/'LATEST.txt').write_text(str(run.resolve()),encoding='utf-8')
    cmd=[sys.executable,'-u',str(run/'code/stride_lab.py'),'--supervisor','--run',str(run),'--hours',str(args.hours),
        '--trials',str(args.trials),'--seed',str(args.seed),'--ratio',str(args.ratio),'--min-mm-s',str(args.min_mm_s)]
    if args.no_alert:cmd+=['--no-alert']
    if args.quick_test:cmd+=['--quick-test']
    with (run/'supervisor.log').open('w',encoding='utf-8') as log:
        kw=dict(cwd=ROOT,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT)
        if os.name=='nt':kw['creationflags']=subprocess.DETACHED_PROCESS|subprocess.CREATE_NEW_PROCESS_GROUP
        else:kw['start_new_session']=True
        p=subprocess.Popen(cmd,**kw)
    print('STARTED SUPERVISOR',p.pid,'WORKERS',info['workers'],'ALL REQUESTED DIRECTIONS',len(dirs))
    print('RUN',run);print('Local completion alarm enabled:',not args.no_alert)
    print('This console may close. Keep PC powered and logged in. No reboot recovery.')

if __name__=='__main__':
    import importlib.metadata
    main()
