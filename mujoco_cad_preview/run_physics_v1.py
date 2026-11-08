"""Step BGH Physics V1 under gravity and save diagnostic measurements.

No hardware I/O, no reinforcement learning, and no standing guarantees.
The CSV-like JSON log records root height, tilt and foot contacts.
"""
from __future__ import annotations
import argparse, json, math, sys, time
from pathlib import Path
from xml.etree import ElementTree as ET
import numpy as np
import mujoco
import mujoco.viewer

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'output'

def load_model(xml_path):
    xml=xml_path.read_text(encoding='utf-8')
    root=ET.fromstring(xml)
    assets=root.findall('asset/mesh')
    if not assets:
        return mujoco.MjModel.from_xml_string(xml)
    # Read raw STL bytes using Python: handles Korean directory names.
    with mujoco.MjVfs() as vfs:
        for mesh in assets:
            name=mesh.get('file')
            if not name:raise ValueError('Unnamed CAD mesh asset')
            file=OUT/'cad_meshes'/name
            if not file.is_file():raise FileNotFoundError(str(file))
            vfs[name]=file.read_bytes()
        return mujoco.MjModel.from_xml_string(xml,vfs=vfs)

def tilt_deg(q):
    w,x,y,z=[float(t) for t in q]
    return math.degrees(math.acos(max(-1.,min(1.,1.-2*(x*x+y*y)))))

def contact_feet(model,data):
    names=set()
    for i in range(data.ncon):
        c=data.contact[i]
        for idx in (c.geom1,c.geom2):
            name=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,int(idx)) or ''
            if name.startswith('foot_collision_'):names.add(name)
    return sorted(names)

def snapshot(model,data):
    return {'time_s':float(data.time),'base_height_m':float(data.qpos[2]),
      'body_tilt_deg':float(tilt_deg(data.qpos[3:7])),
      'contact_pairs':int(data.ncon),'foot_contacts':contact_feet(model,data)}

def simulate(model,data,seconds,viewer=None):
    frames=[]
    steps=max(1,int(seconds/model.opt.timestep))
    stride=max(1,int(.05/model.opt.timestep))
    start=time.perf_counter()
    for i in range(steps):
        # Zero setpoint = zero in CAD model, NOT actual servo electrical neutral
        data.ctrl[:]=0
        if viewer:
            with viewer.lock():mujoco.mj_step(model,data)
        else:mujoco.mj_step(model,data)
        if i%stride==0 or i==steps-1:
            frames.append(snapshot(model,data))
            if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
                raise RuntimeError('NaN/Inf state at {:.4f}s'.format(data.time))
        if viewer and i%5==0:
            viewer.sync()
            dt=(i+1)*model.opt.timestep-(time.perf_counter()-start)
            if dt>0:time.sleep(min(dt,.03))
            if not viewer.is_running():break
    return frames

def analyze(model,frames):
    if not frames:raise ValueError('No simulation frames')
    last=[x for x in frames if x['time_s']>=max(0,frames[-1]['time_s']-1.)]
    low=min(x['base_height_m'] for x in last)
    tilt=max(x['body_tilt_deg'] for x in last)
    feet=max(len(x['foot_contacts']) for x in frames)
    verdict=('PRELIMINARY_STAND_CANDIDATE'
             if low>.075 and tilt<25 and feet>=2
             else 'NOT_STANDING_YET')
    return {
      'mujoco_version':mujoco.__version__,'model_nq':int(model.nq),
      'model_nv':int(model.nv),'actuators':int(model.nu),
      'gravity_m_s2':model.opt.gravity.tolist(),
      'physics_stepping':True,'rl_training':False,
      'simulated_seconds':float(frames[-1]['time_s']),
      'last_second_min_height_m':low,'last_second_max_tilt_deg':tilt,
      'maximum_simultaneous_foot_contacts':feet,'final_state':frames[-1],
      'heuristic_verdict':verdict,
      'warning':('UNVERIFIED mass/servo/foot physics; a preliminary candidate '
                 'is NOT proof of stable real-world standing.'),
      'samples':frames,
    }

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds',type=float,default=3.)
    p.add_argument('--view',action='store_true',
                   help='Open MuJoCo viewer with real gravity stepping')
    args=p.parse_args()
    if not .1<=args.seconds<=20:p.error('--seconds must be 0.1..20')
    xml=OUT/'physics_v1.xml'
    if not xml.is_file():
        raise FileNotFoundError('Run build_physics_v1.py first')
    model=load_model(xml)
    data=mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    if (model.nq,model.nv,model.nu)!=(35,34,12):
        raise RuntimeError('Unexpected free-base dimensions; expected 35/34/12')
    if args.view:
        print('Physics ON. Robot may fall; NOT trained standing/walking.')
        with mujoco.viewer.launch_passive(model,data) as viewer:
            viewer.cam.lookat[:]=[0,0,.08]
            viewer.cam.distance=.62
            viewer.cam.azimuth=135
            viewer.cam.elevation=-18
            frames=simulate(model,data,args.seconds,viewer)
            report=analyze(model,frames)
            (OUT/'physics_v1_report.json').write_text(
                json.dumps(report,indent=2),encoding='utf-8')
            while viewer.is_running():
                viewer.sync()
                time.sleep(.04)
    else:
        frames=simulate(model,data,args.seconds)
        report=analyze(model,frames)
        (OUT/'physics_v1_report.json').write_text(
            json.dumps(report,indent=2),encoding='utf-8')
    fields=('mujoco_version','model_nq','model_nv','actuators',
      'simulated_seconds','last_second_min_height_m','last_second_max_tilt_deg',
      'maximum_simultaneous_foot_contacts','final_state','heuristic_verdict')
    print(json.dumps({k:report[k] for k in fields},indent=2,ensure_ascii=False))
    print('Detailed diagnostics:',OUT/'physics_v1_report.json')

if __name__=='__main__':
    try:main()
    except Exception:
        import traceback
        traceback.print_exc()
        sys.exit(1)
