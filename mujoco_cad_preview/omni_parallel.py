"""Bounded PARALLEL multidirectional crawl optimization. NOT PPO. NO HARDWARE.
Start detached on Windows: python omni_parallel.py --start --hours 12 --workers 0
Status: ... --status   Stop: ... --stop   Replay: ... --view left
Each worker is a separate process, BLAS single-threaded, logs redirected to files.
"""
from __future__ import annotations
import os
for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS'):os.environ[k]='1'
import argparse, csv, ctypes, hashlib, importlib.metadata, json, math, random, shutil, subprocess, sys, time, traceback, zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parent
FILES=('linkage_core.py','omni_trial.py','omni_parallel.py')


def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def write(p,x):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True);tmp=p.with_suffix(p.suffix+'.tmp')
    tmp.write_text(json.dumps(x,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf-8');tmp.replace(p)
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def lock_file(path):
    path.parent.mkdir(parents=True,exist_ok=True);f=path.open('a+b');f.seek(0)
    if not f.read(1):f.write(b'0');f.flush()
    f.seek(0)
    try:
        if os.name=='nt':
            import msvcrt;msvcrt.locking(f.fileno(),msvcrt.LK_NBLCK,1)
        else:
            import fcntl;fcntl.flock(f,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except OSError:
        f.close();raise RuntimeError('Another optimizer is running. Do not start two masters.')
    return f


def resources(request):
    cores=getattr(os,'process_cpu_count',os.cpu_count)() or 1;available=None
    if os.name=='nt':
        class MEMORY(ctypes.Structure):
            _fields_=[('length',ctypes.c_ulong),('load',ctypes.c_ulong)]+[(n,ctypes.c_ulonglong) for n in
                ('total','avail','page','avail_page','virtual','avail_virtual','extended')]
        mem=MEMORY();mem.length=ctypes.sizeof(mem)
        if ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(mem)):available=mem.avail/1024**3
    else:
        try:
            for line in Path('/proc/meminfo').read_text().splitlines():
                if line.startswith('MemAvailable:'):available=int(line.split()[1])/1024**2
        except OSError:pass
    cap=max(1,min(8,cores-2 if cores>2 else 1))
    if available is not None:cap=min(cap,max(1,int((available-1.5)/.75)))
    auto=max(1,min(4,cores//2,cap))
    if request>cap:raise ValueError(f'Requested {request} workers exceeds guarded cap {cap}; use --workers 0.')
    return dict(logical_cpus=cores,available_ram_GB=available,workers=request or auto,
                threads_per_worker=1,auto_worker_cap=4,hard_worker_cap=cap)


def seed_params(direction,default):
    p=dict(default);p['stride_mm']=16. if direction in ('forward','backward') else 6.
    p['yaw_deg']=1.;return p


def compare(report,base):
    if not report.get('passed') or not base.get('passed'):return None
    a,b=report['segments'][0],base['segments'][0]
    if a['command']!=b['command'] or a['cycles']!=b['cycles'] or report['friction']!=base['friction']:
        raise ValueError('Comparison requires identical command/cycle/contact conditions')
    rotate=a['command'].startswith('turn')
    speed=abs(a['yaw_deg_s'])/max(abs(b['yaw_deg_s']),1e-12) if rotate else a['speed_mm_s']/max(b['speed_mm_s'],1e-12)
    work=(a['work_J']/max(abs(a['actual_yaw_rad']),1e-9))/(b['work_J']/max(abs(b['actual_yaw_rad']),1e-9)) if rotate else a['mechanical_cot_proxy']/max(b['mechanical_cot_proxy'],1e-12)
    sway=a['lateral_sway_pp_mm']/max(b['lateral_sway_pp_mm'],1.)
    tilt=a['rms_tilt_deg']/max(b['rms_tilt_deg'],.2)
    accel=a['acceleration_rms_mps2']/max(b['acceleration_rms_mps2'],1e-6)
    stable=bool(a['max_tilt_deg']<=min(10.,b['max_tilt_deg']+1.) and a['placement_max_mm']<8)
    improved=bool(stable and work<=1.05 and ((speed>=1.10 and sway<=1.05 and accel<=1.10) or
                    (speed>=.9 and sway<=.85 and accel<=1.05)))
    return dict(speed_ratio=speed,work_ratio=work,sway_ratio=sway,tilt_rms_ratio=tilt,accel_ratio=accel,
        improved=improved,score=float(speed-.5*max(0,sway-1)-.3*max(0,work-1)-.2*max(0,accel-1)))


class Batch:
    def __init__(self,run,workers,deadline):
        self.run=run;self.workers=workers;self.deadline=deadline;self.done=0;self.total=0;self.phase='init';self.errors=0
    def event(self,kind,**data):
        with (self.run/'events.jsonl').open('a',encoding='utf-8') as f:
            f.write(json.dumps(dict(event=kind,time=time.time(),**data))+'\n')
    def update(self,live):
        write(self.run/'status.json',dict(state='RUNNING',phase=self.phase,pid=os.getpid(),updated=time.time(),
            workers=self.workers,finished_jobs=self.done,submitted_jobs=self.total,
            active=[dict(id=x['name'],pid=x['proc'].pid,elapsed_s=round(time.monotonic()-x['start'],1),
                         log=str(x['out']/'console.log')) for x in live]))
    def jobs(self,specs,phase,until=None):
        self.phase=phase;queue=list(specs);live=[];results={};self.total+=len(queue);heartbeat=0.
        limit=min(until or self.deadline,self.deadline)
        try:
            while queue or live:
                if (self.run/'STOP').exists():raise KeyboardInterrupt('STOP file')
                if time.monotonic()>limit:break
                while queue and len(live)<self.workers:
                    spec=queue.pop(0);name=spec.pop('_id');out=self.run/name;out.mkdir(parents=True,exist_ok=True)
                    if (out/'report.json').exists():results[name]=read(out/'report.json');self.done+=1;continue
                    task=dict(spec,source=str(self.run/'source.xml'),out=str(out));write(out/'task.json',task)
                    log=(out/'console.log').open('w',encoding='utf-8')
                    proc=subprocess.Popen([sys.executable,'-u',str(self.run/'code/omni_trial.py'),str(out/'task.json')],
                        cwd=self.run/'code',stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,
                        creationflags=subprocess.CREATE_NO_WINDOW if os.name=='nt' else 0)
                    live.append(dict(name=name,proc=proc,out=out,log=log,start=time.monotonic()))
                    self.event('start',job=name,pid=proc.pid)
                    print('START',name,'PID',proc.pid,flush=True)
                for x in live[:]:
                    timeout=time.monotonic()-x['start']>1200
                    if timeout and x['proc'].poll() is None:x['proc'].terminate()
                    if x['proc'].poll() is not None or timeout:
                        try:x['proc'].wait(timeout=5)
                        except subprocess.TimeoutExpired:x['proc'].kill();x['proc'].wait()
                        x['log'].close();live.remove(x);self.done+=1
                        if (x['out']/'report.json').exists():r=read(x['out']/'report.json')
                        else:
                            r=dict(passed=False,failure='TIMEOUT' if timeout else 'WORKER_ERROR',segments=[])
                            write(x['out']/'report.json',r);self.errors+=1
                        self.event('end',job=x['name'],pid=x['proc'].pid,passed=r['passed'])
                        results[x['name']]=r;print('END',x['name'],r['passed'],flush=True)
                if time.monotonic()-heartbeat>3:self.update(live);heartbeat=time.monotonic()
                time.sleep(.2)
        finally:
            for x in live:
                if x['proc'].poll() is None:
                    x['proc'].terminate()
                    try:x['proc'].wait(timeout=5)
                    except subprocess.TimeoutExpired:x['proc'].kill();x['proc'].wait()
                x['log'].close();self.event('end',job=x['name'],pid=x['proc'].pid,passed=False,interrupted=True)
                write(x['out']/'interrupted.json',dict(status='BUDGET_OR_USER_STOP'))
            self.update([])
        return results


def archive(run,status,selections,search):
    write(run/'selection.json',selections);write(run/'search.json',search)
    old=read(run/'status.json') if (run/'status.json').exists() else {}
    write(run/'status.json',dict(old,state=status,updated=time.time(),active=[]))
    lines=['# Parallel multidirectional crawl experiments','Status: '+status,
        'Controller parameter optimization, NOT PPO or neural network training.',
        'No old weights/physics overwritten. Full simulator state; not hardware ready.',
        'Only confirmed entries below may be replayed; a failure remains a failure.','']
    for d,s in selections.items():lines.append(d+': '+s['label'])
    (run/'SUMMARY.md').write_text('\n'.join(lines),encoding='utf-8')
    with (run/'ranking.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.writer(f);w.writerow(['trial','direction','passed','speed_ratio','work_ratio','sway_ratio','score'])
        for x in search:
            c=x.get('comparison') or {};w.writerow([x['id'],x['direction'],x['passed'],*[c.get(k) for k in ('speed_ratio','work_ratio','sway_ratio','score')]])
    temp=run/'results_bundle.tmp.zip'
    with zipfile.ZipFile(temp,'w',zipfile.ZIP_DEFLATED) as z:
        for p in run.rglob('*'):
            if p.is_file() and p.suffix in ('.json','.jsonl','.md','.csv','.py','.xml'):
                z.write(p,str(p.relative_to(run)))
    temp.replace(run/'results_bundle.zip')


def optimize(run,args):
    from omni_trial import DEFAULT,BOUNDS,DIRECTIONS
    manifest=read(run/'manifest.json');meta=manifest['resources'];deadline=time.monotonic()+args.hours*3600
    batch=Batch(run,meta['workers'],deadline);rng=random.Random(args.seed);selections={};search=[];awake=False
    status='COMPLETED';confirmed={};lock=None
    try:
        lock=lock_file(run.parent/'RUNNING.lock')
        # Do not compete silently with the previous overnight optimizer.
        old_lock=Path(manifest.get('application_root',str(ROOT)))/'output/crawl_night/RUNNING.lock'
        if old_lock.exists():probe=lock_file(old_lock);probe.close()
        if os.name=='nt':awake=bool(ctypes.windll.kernel32.SetThreadExecutionState(0x80000001))
        print('PARALLEL OMNI START',run,'workers',meta['workers'],'NOT PPO',flush=True)
        names=args.commands.split(',') if args.commands else [d for d in DIRECTIONS if d!='stop']
        if len(set(names))!=len(names) or any(n not in DIRECTIONS or n=='stop' for n in names):raise ValueError('Invalid commands')
        nc=1 if args.quick_test else 3
        spec=lambda name,d,p,n,mu=1.:dict(_id=name,direction=d,params=p,cycles=n,friction=mu)
        seeds={d:seed_params(d,DEFAULT) for d in names}
        refs=batch.jobs([spec('reference_'+d,d,seeds[d],nc) for d in names],'REFERENCE')
        eligible=[d for d in names if refs.get('reference_'+d,{}).get('passed')]
        for d in names:selections[d]=dict(label='UNVERIFIED_NO_PASSED_REFERENCE',run_dir=None)
        if not eligible:status='NO_REFERENCE_GAIT_NO_SEARCH';return
        ref1=refs if nc==1 else batch.jobs([spec('screenref_'+d,d,seeds[d],1) for d in eligible],'SCREEN_REFERENCE')
        baseline={d:ref1.get(('reference_' if nc==1 else 'screenref_')+d) for d in eligible}
        eligible=[d for d in eligible if baseline[d] and baseline[d]['passed']]
        search_until=deadline-min(7200.,args.hours*3600*.4)
        elite={d:[(1.,seeds[d])] for d in eligible}
        count=2 if args.quick_test else args.trials
        for offset in range(0,count,max(1,meta['workers']*2)):
            if (not args.quick_test and time.monotonic()>search_until) or not eligible:break
            tasks=[];used={}
            for i in range(offset,min(count,offset+meta['workers']*2)):
                d=eligible[i%len(eligible)];p=dict(rng.choice(elite[d][:4])[1])
                keys=['stride_mm','shift_s','swing_scale','travel_weight','curve'] if not d.startswith('turn') else ['yaw_deg','shift_s','travel_weight','curve']
                for key in rng.sample(keys,1 if i<len(eligible)*3 else 2):
                    lo,hi=BOUNDS[key]
                    if key=='curve':p[key]=3 if p[key]==5 else 5
                    else:p[key]=round(float(np_clip(p[key]+rng.gauss(0.,(hi-lo)*.15),lo,hi)),4)
                name=f'trial_{i:04d}_{d}';tasks.append(spec(name,d,p,1));used[name]=(d,p)
            result=batch.jobs(tasks,'SEARCH',deadline if args.quick_test else search_until)
            for name,r in result.items():
                d,p=used[name];c=compare(r,baseline[d]);search.append(dict(id=name,direction=d,params=p,passed=r['passed'],comparison=c))
                if c:
                    elite[d].append((c['score'],p));elite[d].sort(key=lambda x:x[0],reverse=True)
            write(run/'search.json',search)
            if batch.errors>=3:status='WORKER_ERRORS_STOP';break
        if args.quick_test:status='SMOKE_ONLY_NOT_QUALIFIED';return
        finalists={}
        for d in eligible:
            best=sorted([x for x in search if x['direction']==d and (x.get('comparison') or {}).get('improved')],
                        key=lambda x:x['comparison']['score'],reverse=True)[:2]
            for k,x in enumerate(best):finalists['final_'+d+'_'+str(k)]=(d,x['params'],True)
            finalists['final_'+d+'_reference']=(d,seeds[d],False)
        checks=batch.jobs([spec(name,d,p,6) for name,(d,p,imp) in finalists.items()],'SIX_CYCLE_CONFIRMATION')
        holdtasks=[]
        for name,r in checks.items():
            if r['passed']:
                d,p,imp=finalists[name]
                for mu in (.9,1.1):holdtasks.append(spec(name+'_mu'+str(mu),d,p,3,mu))
        holds=batch.jobs(holdtasks,'FRICTION_HOLDOUT')
        for name,r in checks.items():
            if not r['passed'] or not all(holds.get(name+'_mu'+str(mu),{}).get('passed') for mu in (.9,1.1)):continue
            d,p,imp=finalists[name];refname='final_'+d+'_reference';reference=checks.get(refname,{})
            cmp=compare(r,reference) if reference.get('passed') else None
            # Improvement also must hold at matching perturbed contact conditions.
            hold_cmp=[compare(holds[name+'_mu'+str(mu)],holds[refname+'_mu'+str(mu)])
                      if holds.get(refname+'_mu'+str(mu),{}).get('passed') else None for mu in (.9,1.1)]
            qualified_improvement=imp and cmp and cmp['improved'] and all(c and c['improved'] for c in hold_cmp)
            if imp and not qualified_improvement:continue
            score=(cmp or {}).get('score',1.)
            if d not in confirmed or score>confirmed[d][0]:
                confirmed[d]=(score,p)
                selections[d]=dict(label='CONFIRMED_IMPROVEMENT' if qualified_improvement else 'CONFIRMED_REFERENCE_ONLY',
                    run_dir=str(run/name),params=p,comparison=cmp,friction_comparisons=hold_cmp)
        chain=[d for d in ['forward','left','forward_left','backward','right','backward_right','turn_left','turn_right'] if d in confirmed]
        if len(chain)>=2:
            chain.insert(len(chain)//2,'stop');chain.append('stop')
            cp={d:confirmed[d][1] for d in chain if d!='stop'};cp['stop']=dict(DEFAULT)
            trans=dict(_id='transition_sequence',direction=chain[0],commands=chain,command_params=cp,params=cp[chain[0]],cycles=1)
            r=batch.jobs([trans],'COMMAND_TRANSITIONS').get('transition_sequence',{})
            selections['sequence']=dict(label='CONFIRMED_SEQUENCE' if r.get('passed') else 'SEQUENCE_NOT_CONFIRMED',
                run_dir=str(run/'transition_sequence') if r.get('passed') else None,commands=chain)
        status='CONFIRMED_DIRECTIONS_SAVED' if confirmed else 'NO_CONFIRMED_DIRECTION_NO_DEPLOYMENT'
    except KeyboardInterrupt:status='STOPPED_CHECKPOINTS_PRESERVED'
    except Exception:
        status='BATCH_ERROR';(run/'error.txt').write_text(traceback.format_exc(),encoding='utf-8');traceback.print_exc()
    finally:
        if sha(run/'source.xml')!=manifest['source_sha256']:
            status='SOURCE_CHANGED_NO_DEPLOYMENT';selections={}
        archive(run,status,selections,search)
        if awake:ctypes.windll.kernel32.SetThreadExecutionState(0x80000000)
        if lock:lock.close()
        print('FINISHED',status,'SHARE',run/'results_bundle.zip',flush=True)


def np_clip(x,lo,hi):return max(lo,min(hi,x))


def replay(run,direction):
    import numpy as np
    import mujoco
    import mujoco.viewer
    from xml.etree import ElementTree as ET
    choices=read(run/'selection.json');choice=choices.get(direction,{})
    if not choice.get('run_dir'):raise RuntimeError('No confirmed result for '+direction+'; read SUMMARY.md.')
    out=Path(choice['run_dir']);report=read(out/'report.json')
    if not report.get('passed'):raise RuntimeError('Failed trajectory cannot be shown as passed')
    root=ET.fromstring((run/'source.xml').read_text('utf-8-sig'));original=Path(read(run/'manifest.json')['source_original'])
    assets={}
    for mesh in root.findall('asset/mesh'):
        name=mesh.get('file');candidates=[original.parent/'cad_meshes'/name,ROOT/'output/cad_meshes'/name]
        path=next((p for p in candidates if p.is_file()),None)
        if path is None:raise FileNotFoundError('Missing visual mesh '+name)
        assets[name]=path.read_bytes()
    model=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'),assets=assets);data=mujoco.MjData(model)
    with np.load(out/'trajectory.npz',allow_pickle=False) as z:q=z['qpos'].copy();dt=float(z['dt'])
    print('1X RECORDED PHYSICS REPLAY; NOT A NEW SIMULATION. '+choice['label'],flush=True)
    with mujoco.viewer.launch_passive(model,data) as viewer:
        viewer.cam.distance=.65;viewer.cam.azimuth=145;viewer.cam.elevation=-20;viewer.cam.lookat[:]=[0,0,.06]
        start=time.perf_counter();old=-1
        while viewer.is_running():
            i=int((time.perf_counter()-start)/dt)%len(q)
            if i!=old:
                with viewer.lock():data.qpos[:]=q[i];data.qvel[:]=0.;mujoco.mj_forward(model,data)
                viewer.sync();old=i
            time.sleep(.005)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--hours',type=float,default=12.);ap.add_argument('--workers',type=int,default=0)
    ap.add_argument('--trials',type=int,default=240);ap.add_argument('--seed',type=int,default=20261117)
    ap.add_argument('--source',type=Path,default=ROOT/'output/physics_v1.xml');ap.add_argument('--commands',default='')
    ap.add_argument('--start',action='store_true');ap.add_argument('--run-dir',type=Path)
    ap.add_argument('--status',action='store_true');ap.add_argument('--stop',action='store_true');ap.add_argument('--view')
    ap.add_argument('--quick-test',action='store_true')
    args=ap.parse_args();folder=ROOT/'output/omni_parallel';folder.mkdir(parents=True,exist_ok=True)
    if args.status or args.stop or args.view:
        run=Path((folder/'LATEST.txt').read_text('utf-8').strip())
        if args.stop:(run/'STOP').write_text('User requested stop');print('STOP requested:',run);return
        if args.view:replay(run,args.view);return
        print('RUN:',run)
        print(json.dumps(read(run/'status.json'),indent=2,ensure_ascii=False))
        if (run/'master.log').exists():print('\n'.join((run/'master.log').read_text('utf-8',errors='replace').splitlines()[-12:]))
        return
    if not .05<=args.hours<=72 or not 0<=args.workers<=8 or not 1<=args.trials<=1000:ap.error('Invalid time/workers/trials limits')
    if args.run_dir:optimize(args.run_dir,args);return
    if not args.source.is_file():raise FileNotFoundError('Existing physics_v1.xml required')
    probe=lock_file(folder/'RUNNING.lock');probe.close()
    meta=resources(args.workers);run=folder/(time.strftime('%Y%m%d_%H%M%S')+f'_{os.getpid()}');run.mkdir()
    (run/'code').mkdir()
    for name in FILES:shutil.copy2(ROOT/name,run/'code'/name)
    shutil.copy2(args.source,run/'source.xml')
    write(run/'manifest.json',dict(source_original=str(args.source.resolve()),application_root=str(ROOT),source_sha256=sha(args.source),
        code_sha256={name:sha(ROOT/name) for name in FILES},resources=meta,hours=args.hours,seed=args.seed,
        versions={name:importlib.metadata.version(name) for name in ('mujoco','numpy','scipy')},
        parameters_only=True,PPO=False,original_files_modified=False,hardware_control=False))
    (folder/'LATEST.txt').write_text(str(run.resolve()),encoding='utf-8')
    write(run/'status.json',dict(state='STARTING',updated=time.time(),workers=meta['workers']))
    if args.start:
        cmd=[sys.executable,'-u',str(run/'code/omni_parallel.py'),'--run-dir',str(run.resolve()),
             '--hours',str(args.hours),'--workers',str(meta['workers']),'--trials',str(args.trials),'--seed',str(args.seed)]
        if args.quick_test:cmd.append('--quick-test')
        if args.commands:cmd+=['--commands',args.commands]
        with (run/'master.log').open('w',encoding='utf-8') as log:
            kw=dict(cwd=ROOT,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT)
            if os.name=='nt':kw['creationflags']=subprocess.DETACHED_PROCESS|subprocess.CREATE_NEW_PROCESS_GROUP
            else:kw['start_new_session']=True
            proc=subprocess.Popen(cmd,**kw)
        write(run/'launch.json',dict(pid=proc.pid,detached=True));print('STARTED PID',proc.pid,'WORKERS',meta['workers'])
        print('RUN:',run);print('This shell can close. Keep PC powered, ventilated and awake.')
        print('STATUS: .venv\\Scripts\\python.exe omni_parallel.py --status')
    else:optimize(run,args)

if __name__=='__main__':main()
