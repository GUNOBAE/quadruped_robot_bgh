"""Physics V1: add gravity, contacts and provisional inertias to CAD linkage.

NOT calibrated, NOT verified standing, NOT reinforcement learning.
STL meshes remain visual-only; collisions use simple torso/foot shapes.
"""
from __future__ import annotations
import argparse, json, struct
from collections import defaultdict
from pathlib import Path
from xml.etree import ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'output'
STL_DTYPE = np.dtype([('normal','<f4',(3,)),('vertex','<f4',(3,3)),('attrib','<u2')])
PROXY_MASS = {
 'base_link':1.5325, 'front v1':.3239, 'Raspberrypiwall':.4291,
 'Back':.2419, 'Charger_wall':.2896, 'FL_Coxa_motor':.1522,
 'FR_Coxa_motor':.1522, 'BL_Coxa_motor':.1522, 'BR_Coxa_motor':.1522,
}

def weight(name):
    short=name.rsplit(':',1)[0]
    for prefix,mass in PROXY_MASS.items():
        if short==prefix or short.startswith(prefix+' '):
            return mass
    lower=short.lower()
    if lower.startswith(('fl_a','fr_a','bl_a','br_a')):
        return .3918
    for prefix,mass in (
      ('servo_mount',.00844),('servo_arm',.03432),
      ('rodend_100mm',.02114),('rodend_5mm',.00534),
      ('femur',.26834),('tibia',.20518)):
        if lower.startswith(prefix):return mass
    raise ValueError('No provisional mass for '+name)

def triangles(filename):
    data=filename.read_bytes()
    if len(data)<84:raise ValueError('STL too small: '+str(filename))
    n=struct.unpack_from('<I',data,80)[0]
    if len(data)!=84+STL_DTYPE.itemsize*n or not n:
        raise ValueError('Expected nonempty binary STL: '+str(filename))
    return np.frombuffer(data,dtype=STL_DTYPE,offset=84,count=n)['vertex'].reshape(-1,3).astype(float)

def words(v):
    return ' '.join('{:.10g}'.format(float(x)) for x in v)

def body_table(xml_root):
    bodies,origins={},{}
    def visit(body,parent):
        name=body.get('name')
        if not name or name in bodies:raise ValueError('Bad body name '+str(name))
        if any(body.get(k) for k in ('quat','euler','axisangle')):
            raise ValueError('Rotated preview body '+name)
        o=parent+np.fromstring(body.get('pos','0 0 0'),sep=' ')
        bodies[name]=body
        origins[name]=o
        for child in body.findall('body'):visit(child,o)
    for body in xml_root.findall('./worldbody/body'):
        visit(body,np.zeros(3))
    return bodies,origins

def add_inertial(body,parts,mass_scale):
    m=np.array([p['mass'] for p in parts],float)
    centers=np.array([p['center'] for p in parts],float)
    com=np.average(centers,axis=0,weights=m)
    I=np.zeros(3)
    for x,p in zip(m,parts):
        d=np.maximum(p['hi']-p['lo'],.005)
        offset=p['center']-com
        I+=np.array([
          x*(d[1]**2+d[2]**2)/12+x*(offset[1]**2+offset[2]**2),
          x*(d[0]**2+d[2]**2)/12+x*(offset[0]**2+offset[2]**2),
          x*(d[0]**2+d[1]**2)/12+x*(offset[0]**2+offset[1]**2)])
    I=np.maximum(I*mass_scale,1e-8)
    mass=float(sum(m)*mass_scale)
    ET.SubElement(body,'inertial',pos=words(com),
       mass='{:.9g}'.format(mass),diaginertia=words(I))
    return {'mass_kg':mass,'center_local_m':com.tolist(),
            'diag_inertia_kg_m2':I.tolist()}

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--total-mass-kg',type=float,default=None,
       help='Measured total mass; absent means unverified CAD-derived ~7.16kg')
    ap.add_argument('--torque-limit-nm',type=float,default=2.5,
       help='Placeholder output torque limit per servo, NOT verified')
    ap.add_argument('--kp',type=float,default=20)
    ap.add_argument('--kv',type=float,default=1.)
    ap.add_argument('--toe-radius-m',type=float,default=.009)
    ap.add_argument('--initial-clearance-m',type=float,default=.007)
    ap.add_argument('--schematic',action='store_true',
       help='Use old proxy shapes without real CAD; CI tests only')
    args=ap.parse_args()
    if args.total_mass_kg is not None and args.total_mass_kg<=0:
        ap.error('Measured total mass must be positive')
    if args.torque_limit_nm<=0 or args.kp<=0 or args.kv<0:
        ap.error('Invalid servo torque or gains')
    path=OUT/('quadruped_preview.xml' if args.schematic else 'cad_preview.xml')
    if not path.is_file():raise FileNotFoundError('Run NEW_PC_START.cmd first: '+str(path))
    root=ET.parse(path).getroot()
    root.set('model','BGH_Physics_V1_UNCALIBRATED_NOT_RL')
    bodies,origins=body_table(root)
    mapping=json.loads((OUT/'model_manifest.json').read_text('utf-8'))['body_names']
    rigid=json.loads((ROOT/'audit_excerpt_reference.json').read_text('utf-8'))['rigid_groups'][0]['occurrences']
    by_name={name:mapping['BASE'] for name in rigid}
    by_name.update({name:bid for name,bid in mapping.items() if name!='BASE'})
    if len(bodies)!=29 or len(by_name)!=37:
        raise RuntimeError('Expected 29 body nodes and 37 CAD part occurrences')
    parts=defaultdict(list)
    feet={}
    if not args.schematic:
        report=json.loads((OUT/'cad_fit_report.json').read_text('utf-8'))
        if report.get('errors') or len(report.get('parts',[]))!=37:
            raise ValueError('Incomplete STL CAD import. Fix it before physics.')
        for i,item in enumerate(report['parts']):
            name=item['occurrence']
            bid=by_name[name]
            verts=triangles(OUT/'cad_meshes'/('fusion_part_{:02d}.stl'.format(i)))
            lo,hi=verts.min(axis=0),verts.max(axis=0)
            parts[bid].append({'name':name,'mass':weight(name),
               'lo':lo,'hi':hi,'center':(lo+hi)/2})
            if name.lower().startswith('tibia'):
                near=verts[verts[:,2]<=lo[2]+.003]
                if len(near)<1:raise ValueError('No tibia tip geometry '+name)
                feet[bid]=np.array([float(np.median(near[:,0])),
                   float(np.median(near[:,1])),float(lo[2])+args.toe_radius_m])
    else:
        for name,bid in by_name.items():
            coords=[]
            for geom in bodies[bid].findall('geom'):
                if geom.get('fromto'):
                    coords.extend(np.fromstring(geom.get('fromto'),sep=' ').reshape(2,3))
                if geom.get('pos'):
                    coords.append(np.fromstring(geom.get('pos'),sep=' '))
            if not coords:coords=[np.zeros(3)]
            pts=np.asarray(coords,float)
            lo,hi=pts.min(axis=0)-.005,pts.max(axis=0)+.005
            parts[bid].append({'name':name,'mass':weight(name),
               'lo':lo,'hi':hi,'center':(lo+hi)/2})
        for name,bid in by_name.items():
            if name.lower().startswith('tibia'):
                spheres=[g for g in bodies[bid].findall('geom')
                         if g.get('type')=='sphere' and g.get('pos')]
                if not spheres:raise ValueError('No schematic toe for '+name)
                feet[bid]=np.fromstring(spheres[-1].get('pos'),sep=' ')
    if len(feet)!=4:raise ValueError('Need four distinct toe collision bodies')
    total=sum(sum(p['mass'] for p in ps) for ps in parts.values())
    scale=(args.total_mass_kg or total)/total

    base=bodies[mapping['BASE']]
    ET.SubElement(base,'freejoint',name='base_free')
    lowest=min(origins[bid][2]+toe[2]-args.toe_radius_m for bid,toe in feet.items())
    height=-lowest+args.initial_clearance_m
    base.set('pos',words((0,0,height)))
    config={'source':str(path),'source_kind':('SCHEMATIC' if args.schematic else 'CAD_STL'),
      'provisional_cad_mass_kg':total,'selected_total_mass_kg':total*scale,
      'servo_torque_limit_nm':args.torque_limit_nm,'kp':args.kp,'kv':args.kv,
      'toe_radius_m':args.toe_radius_m,'initial_base_height_m':height,
      'bodies':{},'toe_colliders':{},
      'warning':'UNVERIFIED mass, contacts, servo axes, torque and physical stance'}
    for name,body in bodies.items():
        if name not in parts:raise ValueError('Missing inertial part for '+name)
        config['bodies'][name]=add_inertial(body,parts[name],scale)

    floor=root.find('./worldbody/geom[@name="visual_floor"]')
    if floor is None:raise RuntimeError('Schematic floor missing')
    for k,v in {'pos':'0 0 0','size':'.9 .9 .02','rgba':'.84 .84 .84 1',
                'contype':'1','conaffinity':'1','condim':'3',
                'friction':'1.1 .01 .0001','group':'0'}.items():floor.set(k,v)
    for body in bodies.values():
        for geom in body.findall('geom'):
            geom.set('contype','0')
            geom.set('conaffinity','0')
    ET.SubElement(base,'geom',name='torso_collision',type='box',
       pos='0 0 .016',size='.047 .145 .033',friction='.8 .01 .0001',
       condim='3',contype='1',conaffinity='1',group='4',rgba='0 0 0 0')
    for bid,toe in feet.items():
        ET.SubElement(bodies[bid],'geom',name='foot_collision_'+bid,type='sphere',
          pos=words(toe),size=str(args.toe_radius_m),friction='1.1 .01 .0001',
          condim='3',contype='1',conaffinity='1',group='4',rgba='0 0 0 0')
        ET.SubElement(bodies[bid],'site',name='foot_'+bid,pos=words(toe),
          size='.004',group='4',rgba='0 .8 .1 1')
        config['toe_colliders'][bid]={'local_center_m':toe.tolist(),
          'initial_world_bottom_m':float(height+origins[bid][2]+toe[2]-args.toe_radius_m)}
    ET.SubElement(base,'site',name='imu_site',pos='0 0 .02',
                  size='.004',group='4',rgba='0 .3 .8 1')
    sensors=root.find('sensor')
    if sensors is None:sensors=ET.SubElement(root,'sensor')
    ET.SubElement(sensors,'gyro',name='base_gyro',site='imu_site')
    ET.SubElement(sensors,'accelerometer',name='base_accel',site='imu_site')
    ET.SubElement(sensors,'framequat',name='base_quat',
                  objtype='body',objname=mapping['BASE'])
    opt=root.find('option')
    opt.set('gravity','0 0 -9.81')
    opt.set('timestep','.001')
    opt.set('iterations','150')
    opt.set('integrator','implicitfast')
    flag=opt.find('flag')
    if flag is not None:flag.set('contact','enable')
    defaults=root.find('./default/joint')
    if defaults is not None:
        defaults.set('damping','.07')
        defaults.set('armature','.0006')
    acts=root.find('actuator')
    if acts is None or len(acts)!=12:raise RuntimeError('Expected 12 provisional servos')
    for act in acts:
        for k,v in {'kp':str(args.kp),'kv':str(args.kv),
            'ctrlrange':'-.25 .25',
            'forcerange':words((-args.torque_limit_nm,args.torque_limit_nm)),
            'forcelimited':'true','ctrllimited':'true'}.items():act.set(k,v)
    compiler=root.find('compiler')
    if compiler is not None:compiler.set('inertiafromgeom','false')
    OUT.mkdir(exist_ok=True)
    ET.indent(root,space='  ')
    ET.ElementTree(root).write(OUT/'physics_v1.xml',encoding='utf-8',
                                xml_declaration=True)
    (OUT/'physics_v1_config.json').write_text(
       json.dumps(config,indent=2,ensure_ascii=False),encoding='utf-8')
    print('PHYSICS V1 WRITTEN:',OUT/'physics_v1.xml')
    print('Free body + gravity + 4 toe contacts + 12 provisional servo PD controllers.')
    print('CAD proxy mass {:.3f} kg; selected mass {:.3f} kg'.format(total,total*scale))
    print('NOT VALIDATED: real servo calibration or standing stability.')

if __name__=='__main__':
    main()
