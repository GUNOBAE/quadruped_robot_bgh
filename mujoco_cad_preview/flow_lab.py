"""Detached, parallel overlap-gait experiments with local completion alarm.
No PPO, no hardware I/O. Original controller/models remain unchanged.
"""
from __future__ import annotations
import os
for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[k]='1'
import argparse,ctypes,hashlib,json,random,shutil,subprocess,sys,time,traceback,zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parent
FILES=('linkage_core.py','omni_trial.py','omni_parallel.py','flow_trial.py','flow_lab.py','flow_notify.py')

def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def write(p,x):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True);tmp=p.with_suffix(p.suffix+'.tmp')
    tmp.write_text(json.dumps(x,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf-8');tmp.replace(p)
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def comparison(a,b):
    if not a.get('passed') or not b.get('passed'):return None
    if (a['cycles'],a['direction'],a['friction'])!=(b['cycles'],b['direction'],b['friction']):raise ValueError('Unlike trial comparison')
    x,y=a['metrics'],b['metrics']
    speed=x['speed_mm_s']/max(y['speed_mm_s'],1e-9);work=x['work_per_distance']/max(y['work_per_distance'],1e-9)
    # Absolute acceleration still reported; faster gait naturally changes it.
    # Continuity is judged by measured stop/reversal fractions, NOT replay speed.
    okay=bool(speed>=1.5 and work<=1.10 and x['forward_fraction']>=max(.65,y['forward_fraction']+.08)
       and x['stall_fraction']<=min(.12,y['stall_fraction']*.8)
       and x['reverse_fraction']<=min(.25,y['reverse_fraction'])
       and x['lateral_sway_mm']<=y['lateral_sway_mm']*1.10+1
       and x['accel_rms']<=max(.45,y['accel_rms']*1.25) and x['max_tilt_deg']<=8)
    return dict(qualified=okay,speed_ratio=speed,work_ratio=work,
        forward_fraction=x['forward_fraction'],stall_fraction=x['stall_fraction'],reverse_fraction=x['reverse_fraction'],
        acceleration_ratio=x['accel_rms']/max(y['accel_rms'],1e-9),
        score=speed+2*x['forward_fraction']-2*x['reverse_fraction']-x['stall_fraction'])


class Queue:
    def __init__(self,run,workers,deadline):self.run=run;self.workers=workers;self.deadline=deadline;self.done=0;self.submitted=0
    def run_jobs(self,jobs,phase,until=None):
        queue=list(jobs);active=[];results={};self.submitted+=len(queue);last=0.
        end=min(until or self.deadline,self.deadline)
        def status():
            write(self.run/'status.json',dict(state='RUNNING',phase=phase,updated=time.time(),pid=os.getpid(),
                finished_jobs=self.done,submitted_jobs=self.submitted,workers=self.workers,
                active=[dict(id=j['id'],pid=j['p'].pid,elapsed_s=time.monotonic()-j['start']) for j in active]))
        try:
            while queue or active:
                if (self.run/'STOP').exists():raise KeyboardInterrupt('STOP requested')
                if time.monotonic()>end:break
                while queue and len(active)<self.workers:
                    spec=queue.pop(0);name=spec['id'];out=self.run/name;out.mkdir(parents=True,exist_ok=True)
                    if (out/'report.json').exists():results[name]=read(out/'report.json');self.done+=1;continue
                    write(out/'task.json',dict(spec,source=str(self.run/'source.xml'),out=str(out)))
                    log=(out/'console.log').open('w',encoding='utf-8')
                    proc=subprocess.Popen([sys.executable,'-u',str(self.run/'code/flow_trial.py'),str(out/'task.json')],
                        cwd=self.run/'code',stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,
                        creationflags=subprocess.CREATE_NO_WINDOW if os.name=='nt' else 0)
                    active.append(dict(id=name,p=proc,start=time.monotonic(),out=out,log=log));print('START',name,proc.pid,flush=True)
                for j in active[:]:
                    timeout=time.monotonic()-j['start']>1800
                    if timeout and j['p'].poll() is None:j['p'].terminate()
                    if j['p'].poll() is None and not timeout:continue
                    try:j['p'].wait(timeout=5)
                    except subprocess.TimeoutExpired:j['p'].kill();j['p'].wait()
                    j['log'].close();active.remove(j);self.done+=1
                    r=read(j['out']/'report.json') if (j['out']/'report.json').exists() else dict(passed=False,failure='WORKER_TIMEOUT_OR_ERROR')
                    results[j['id']]=r;write(j['out']/'report.json',r);print('END',j['id'],r['passed'],flush=True)
                if time.monotonic()-last>3:status();last=time.monotonic()
                time.sleep(.2)
        finally:
            for j in active:
                if j['p'].poll() is None:
                    j['p'].terminate()
                    try:j['p'].wait(timeout=5)
                    except subprocess.TimeoutExpired:j['p'].kill();j['p'].wait()
                j['log'].close();write(j['out']/'interrupted.json',{'reason':'BUDGET_OR_STOP'})
            active=[];status()
        return results


def summarize(run,state,selections,search):
    write(run/'selection.json',selections);write(run/'search.json',search)
    old=read(run/'status.json') if (run/'status.json').exists() else {}
    write(run/'status.json',dict(old,state=state,updated=time.time(),active=[]))
    lines=['# Overlapping crawl experiment','Status: '+state,'NOT PPO. No hardware control.',
           'Only newly confirmed overlaps may be replayed by this tool. Old results are preserved.',
           'Progress is real measured displacement, not animation speed. Mechanical work is NOT electrical energy.','']
    for d,x in selections.items():lines.append(d+': '+x['label'])
    (run/'SUMMARY.md').write_text('\n'.join(lines),encoding='utf-8')
    with zipfile.ZipFile(run/'results_bundle.tmp.zip','w',zipfile.ZIP_DEFLATED) as z:
        for p in run.rglob('*'):
            if p.is_file() and p.suffix in ('.json','.md','.py','.xml'):z.write(p,str(p.relative_to(run)))
    (run/'results_bundle.tmp.zip').replace(run/'results_bundle.zip')
    write(run/'completion.json',dict(state=state,finished_at=time.time(),bundle=str(run/'results_bundle.zip')))


def optimize(run,args):
    from omni_parallel import lock_file
    from flow_trial import DEFAULT,BOUNDS
    from omni_trial import DEFAULT as OLD
    manifest=read(run/'manifest.json');workers=manifest['workers'];deadline=time.monotonic()+args.hours*3600
    batch=Queue(run,workers,deadline);selections={};search=[];lock=None;held=[];awake=False;state='NO_CONFIRMED_FLOW'
    make=lambda ident,d,p,n,ref=False,mu=1.:dict(id=ident,direction=d,params=p,cycles=n,reference=ref,friction=mu)
    try:
        lock=lock_file(run.parent/'RUNNING.lock')
        for name in ('omni_parallel','crawl_night'):
            p=Path(manifest['root'])/'output'/name/'RUNNING.lock'
            if p.exists():held.append(lock_file(p))
        if os.name=='nt':awake=bool(ctypes.windll.kernel32.SetThreadExecutionState(0x80000001))
        dirs=manifest['directions'];rng=random.Random(args.seed)
        base=manifest['reference_params']
        specs=[make('ref_'+d,d,base[d],3,True) for d in dirs]
        refs=batch.run_jobs(specs,'REFERENCE')
        valid=[d for d in dirs if refs.get('ref_'+d,{}).get('passed')]
        for d in dirs:selections[d]=dict(label='UNCONFIRMED_USE_OLD_RESULT',run_dir=None)
        if not valid:state='BASELINE_FAILED_NO_SEARCH';return
        # Known functioning overlap structure is screened before broader search.
        seedjobs=[];params={}
        for d in valid:
            p=dict(DEFAULT,stride_mm=min(20.,max(10.,base[d]['stride_mm']*1.25)))
            params[d]=p;seedjobs.append(make('seed_'+d,d,p,3))
        seeds=batch.run_jobs(seedjobs,'OVERLAP_PREFLIGHT')
        eligible=[d for d in valid if seeds.get('seed_'+d,{}).get('passed')]
        if not eligible:state='OVERLAP_FAILED_NO_LONG_SEARCH';return
        for d in eligible:
            c=comparison(seeds['seed_'+d],refs['ref_'+d]);search.append(dict(id='seed_'+d,direction=d,params=params[d],comparison=c))
        reserve=min(4*3600,args.hours*3600*.4);search_until=deadline-reserve
        elites={d:[((comparison(seeds['seed_'+d],refs['ref_'+d]) or {}).get('score',0),params[d])] for d in eligible}
        for offset in range(0,args.trials,workers*2):
            if time.monotonic()>search_until:break
            jobs=[];data={}
            for i in range(offset,min(args.trials,offset+workers*2)):
                d=eligible[i%len(eligible)];p=dict(rng.choice(elites[d][:4])[1])
                if i<len(eligible):p.update(slot_s=1.2,drift=.4)
                else:
                    for key in rng.sample([k for k in DEFAULT if k!='curve'],2):
                        lo,hi=BOUNDS[key];p[key]=round(max(lo,min(hi,p[key]+rng.gauss(0,(hi-lo)*.12))),4)
                ident=f'trial_{i:04d}_{d}';data[ident]=(d,p);jobs.append(make(ident,d,p,3))
            result=batch.run_jobs(jobs,'SEARCH_CONTINUITY',search_until)
            for name,r in result.items():
                d,p=data[name];c=comparison(r,refs['ref_'+d]);search.append(dict(id=name,direction=d,params=p,comparison=c))
                if c:elites[d].append((c['score'],p));elites[d].sort(key=lambda x:x[0],reverse=True)
            write(run/'search.json',search)
        choices={}
        for d in eligible:
            best=sorted([x for x in search if x['direction']==d and (x.get('comparison') or {}).get('qualified')],key=lambda x:x['comparison']['score'],reverse=True)[:2]
            for k,x in enumerate(best):choices[f'confirm_{d}_{k}']=(d,x['params'])
        if not choices:state='NO_CONTINUITY_IMPROVEMENT';return
        jobs=[make('longref_'+d,d,base[d],12,True) for d in sorted(set(d for d,p in choices.values()))]
        jobs += [make(n,d,p,12) for n,(d,p) in choices.items()]
        long=batch.run_jobs(jobs,'TWELVE_CYCLE_DISTANCE_CHECK')
        jobs=[];approved={}
        for n,(d,p) in choices.items():
            c=comparison(long.get(n,{}),long.get('longref_'+d,{}))
            if c and c['qualified'] and long[n]['metrics']['net_mm']>=120:
                approved[n]=(d,p,c)
                for mu in (.9,1.1):jobs.append(make(n+'_mu'+str(mu),d,p,6,False,mu))
        for d in sorted(set(x[0] for x in approved.values())):
            for mu in (.9,1.1):jobs.append(make('holdref_'+d+'_mu'+str(mu),d,base[d],6,True,mu))
        holds=batch.run_jobs(jobs,'FRICTION_AND_CONTINUITY')
        for n,(d,p,c) in approved.items():
            cc=[comparison(holds.get(n+'_mu'+str(mu),{}),holds.get('holdref_'+d+'_mu'+str(mu),{})) for mu in (.9,1.1)]
            if all(x and x['qualified'] for x in cc):
                prev=selections[d]
                if c['score']>(prev.get('comparison') or {}).get('score',-1e9):
                    selections[d]=dict(label='CONFIRMED_OVERLAP',run_dir=str(run/n),params=p,comparison=c,holdouts=cc)
        state='CONFIRMED_FLOW_SAVED' if any(x.get('run_dir') for x in selections.values()) else 'NO_CONFIRMED_FLOW'
    except KeyboardInterrupt:state='STOPPED_RESULTS_SAVED'
    except Exception:
        state='ERROR_RESULTS_SAVED';(run/'error.txt').write_text(traceback.format_exc(),encoding='utf-8');traceback.print_exc()
    finally:
        if sha(run/'source.xml')!=manifest['source_sha256']:state='SOURCE_CHANGED_NO_RESULT';selections={}
        summarize(run,state,selections,search)
        for f in held:f.close()
        if awake:ctypes.windll.kernel32.SetThreadExecutionState(0x80000000)
        if lock:lock.close()
        print('FINISHED',state,run/'results_bundle.zip',flush=True)


def supervisor(run,args):
    cmd=[sys.executable,'-u',str(run/'code/flow_lab.py'),'--master','--run',str(run),'--hours',str(args.hours),'--trials',str(args.trials),'--seed',str(args.seed)]
    with (run/'master.log').open('w',encoding='utf-8') as log:
        kw=dict(cwd=run/'code',stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT)
        if os.name=='nt':kw['creationflags']=subprocess.CREATE_NO_WINDOW|subprocess.CREATE_NEW_PROCESS_GROUP
        else:kw['start_new_session']=True
        p=subprocess.Popen(cmd,**kw)
        write(run/'master_pid.json',{'pid':p.pid})
        deadline=time.monotonic()+args.hours*3600+180
        while p.poll() is None:
            if time.monotonic()>deadline:
                (run/'STOP').write_text('Supervisor wall-clock limit')
                try:p.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    if os.name=='nt':
                        subprocess.run(['taskkill','/PID',str(p.pid),'/T','/F'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=15)
                    else:
                        import signal
                        os.killpg(p.pid,signal.SIGTERM)
                    p.wait(timeout=10)
                break
            time.sleep(.5)
    if not (run/'completion.json').exists():
        write(run/'completion.json',dict(state='MASTER_EXITED_WITHOUT_FINAL_REPORT',returncode=p.returncode,finished_at=time.time()))
        write(run/'status.json',dict(state='MASTER_EXITED_WITHOUT_FINAL_REPORT',active=[],updated=time.time()))
    # Separate alarm AFTER computation and result finalization. Dismissal cannot
    # block the next optimization trial (there are no remaining trial workers).
    from flow_notify import notify
    notify(run,read(run/'completion.json')['state'],silent=args.no_alert)


def replay(run,d):
    import numpy as np,mujoco,mujoco.viewer
    from linkage_core import load_model
    sel=read(run/'selection.json').get(d,{})
    if not sel.get('run_dir'):raise RuntimeError('No confirmed improvement. Old omni results are unchanged.')
    out=Path(sel['run_dir']);r=read(out/'report.json')
    if not r.get('passed'):raise RuntimeError('Cannot replay failed trial as improvement')
    model,_=load_model(run/'source.xml',visuals=True);data=mujoco.MjData(model)
    with np.load(out/'trajectory.npz',allow_pickle=False) as z:states=z['qpos'].copy();dt=float(z['dt'])
    print('1X PHYSICS RECORDING, NOT LIVE CONTROL. Actual distance mm:',r['metrics']['net_mm'],flush=True)
    with mujoco.viewer.launch_passive(model,data) as v:
        v.cam.distance=.8;v.cam.azimuth=145;v.cam.elevation=-23;v.cam.lookat[:]=[0,0,.07]
        began=time.perf_counter();old=-1
        while v.is_running():
            i=int((time.perf_counter()-began)/dt)%len(states)
            if i!=old:
                with v.lock():data.qpos[:]=states[i];data.qvel[:]=0.;mujoco.mj_forward(model,data)
                v.sync();old=i
            time.sleep(.005)


def main():
    a=argparse.ArgumentParser(description=__doc__)
    a.add_argument('--start',action='store_true');a.add_argument('--hours',type=float,default=12)
    a.add_argument('--workers',type=int,default=0);a.add_argument('--trials',type=int,default=240);a.add_argument('--seed',type=int,default=20261117)
    a.add_argument('--commands',default='forward,backward,forward_right');a.add_argument('--source',type=Path,default=ROOT/'output/physics_v1.xml')
    a.add_argument('--status',action='store_true');a.add_argument('--stop',action='store_true');a.add_argument('--view')
    a.add_argument('--test-alert',action='store_true');a.add_argument('--no-alert',action='store_true')
    a.add_argument('--supervisor',action='store_true');a.add_argument('--master',action='store_true');a.add_argument('--run',type=Path)
    args=a.parse_args();folder=ROOT/'output/flow_lab';folder.mkdir(parents=True,exist_ok=True)
    if args.test_alert:
        from flow_notify import notify
        notify(folder/'alert_test','ALERT_TEST',True,args.no_alert);return
    if args.supervisor:supervisor(args.run,args);return
    if args.master:optimize(args.run,args);return
    if args.status or args.stop or args.view:
        run=Path((folder/'LATEST.txt').read_text('utf-8').strip())
        if args.stop:(run/'STOP').write_text('stop');print('STOP requested');return
        if args.view:replay(run,args.view);return
        print('RUN',run);print(json.dumps(read(run/'status.json'),indent=2,ensure_ascii=False));return
    if not args.start:a.error('Use --start, --status, --view, --stop, or --test-alert')
    if not .05<=args.hours<=48 or not 0<=args.workers<=8 or not 0<=args.trials<=600:a.error('Guarded time/workers/trials exceeded')
    from omni_parallel import resources,lock_file
    from omni_trial import DEFAULT as OLD
    from flow_trial import TRANSLATIONS
    dirs=args.commands.split(',')
    if len(set(dirs))!=len(dirs) or any(d not in TRANSLATIONS for d in dirs):a.error('Translation commands only; yaw remains in previous omni program')
    if not args.source.is_file():raise FileNotFoundError(str(args.source))
    lock=lock_file(folder/'RUNNING.lock');lock.close()
    for f in FILES:
        if not (ROOT/f).is_file():raise FileNotFoundError(f)
    ref={d:dict(OLD,stride_mm=16. if d in ('forward','backward') else 6.,yaw_deg=1.) for d in dirs}
    prior=ROOT/'output/omni_parallel/LATEST.txt';reference_note='conservative original settings'
    if prior.exists():
        chosen=Path(prior.read_text('utf-8').strip());m=read(chosen/'manifest.json')
        if m.get('source_sha256')==sha(args.source):
            for d,x in read(chosen/'selection.json').items():
                if d in ref and x.get('params'):ref[d]=x['params']
            reference_note='same-source selected previous omni settings where available'
    info=resources(args.workers);run=folder/(time.strftime('%Y%m%d_%H%M%S')+f'_{os.getpid()}');run.mkdir();(run/'code').mkdir()
    for f in FILES:shutil.copy2(ROOT/f,run/'code'/f)
    shutil.copy2(args.source,run/'source.xml')
    write(run/'manifest.json',dict(root=str(ROOT),source_original=str(args.source.resolve()),source_sha256=sha(args.source),
        code_sha256={f:sha(ROOT/f) for f in FILES},workers=info['workers'],resources=info,directions=dirs,
        reference_params=ref,reference_note=reference_note,hours=args.hours,PPO=False,hardware=False))
    write(run/'status.json',dict(state='STARTING',updated=time.time(),workers=info['workers']))
    (folder/'LATEST.txt').write_text(str(run.resolve()),encoding='utf-8')
    cmd=[sys.executable,'-u',str(run/'code/flow_lab.py'),'--supervisor','--run',str(run.resolve()),'--hours',str(args.hours),'--trials',str(args.trials),'--seed',str(args.seed)]
    if args.no_alert:cmd.append('--no-alert')
    with (run/'supervisor.log').open('w',encoding='utf-8') as log:
        kw=dict(cwd=ROOT,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT)
        if os.name=='nt':kw['creationflags']=subprocess.DETACHED_PROCESS|subprocess.CREATE_NEW_PROCESS_GROUP
        else:kw['start_new_session']=True
        proc=subprocess.Popen(cmd,**kw)
    print('STARTED SUPERVISOR',proc.pid,'WORKERS',info['workers'],'LOCAL COMPLETION ALARM',not args.no_alert)
    print('RUN',run);print('PowerShell may close. PC must remain powered and logged in; no automatic resume.')

if __name__=='__main__':main()
