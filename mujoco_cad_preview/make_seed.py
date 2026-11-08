"""Recreate the reference 36-joint skeleton from the earlier Fusion audit.
This is a compact manual transcription of anchor points/axes, NOT full CAD geometry.
Units: centimetres. Fixed-base kinematic preview; not a walking model.
"""
import json
from pathlib import Path

LEGS = {
 "FL": {
  "names": ["FL_A v1:1","FL_Coxa_motor:1","Servo_Mount v1:1","rodend_5mm v1:1","servo_arm_mod_L v1:1","femur v1:1","rodend_100mm v1:1","tibia_L v1:1"],
  "foot": [10.6298650641,-11.5854583943,-10.9968056646],
  "joints": [
   (1,"A","C",[6.71365352594,-8.77363456429,4.25],[0,-1,0]),
   (21,"M","A",[9.62721296176,-12.32991065367,2.25001155256],[1,0,0]),
   (22,"H","A",[9.62721296176,-10.32995384373,4.24988628383],[1,0,0]),
   (23,"S","M",[9.88721296176,-14.02676860817,3.94726612459],[1,0,0]),
   (24,"S","H",[9.90221296176,-12.02701011858,5.94694255868],[1,0,0]),
   (25,"F","A",[9.62721296176,-10.32995384373,4.24988628383],[1,0,0]),
   (26,"R","H",[9.90221296176,-8.42076553453,6.15907459303],[1,0,0]),
   (27,"R","T",[9.85221296176,-1.34969772266,-0.91199321883],[1,0,0]),
   (28,"F","T",[11.92721296176,-3.25888603187,-2.82118152804],[1,0,0])
  ]
 },
 "FR": {
  "names": ["FR_A v1:1","FR_Coxa_motor:1","Servo_Mount v1 (1):1","rodend_5mm v2:1","servo_arm_mod_R v1:1","femur v1 (1):1","rodend_100mm v1 (1):1","tibia_R v1:1"],
  "foot": [-9.63900552278,-11.2209938471,-11.3486660123],
  "joints": [
   (3,"A","C",[-5.78634658581,-8.77363455681,4.25],[0,-1,0]),
   (29,"M","A",[-8.66134713646,-12.30863487774,2.25],[1,0,0]),
   (30,"H","A",[-8.66134713646,-10.30863487774,4.25],[-1,0,0]),
   (31,"S","H",[-8.93634713646,-12.00569115258,5.94705627485],[1,0,0]),
   (32,"S","M",[-8.92134713646,-14.00569115258,3.94705627485],[-1,0,0]),
   (33,"F","A",[-8.66134713646,-10.30863487774,4.25],[-1,0,0]),
   (34,"R","H",[-8.93634713646,-8.39944656853,6.15918830920],[1,0,0]),
   (35,"R","T",[-8.86134713646,-1.32837875667,-0.91187950266],[-1,0,0]),
   (36,"T","F",[-10.48634713646,-3.23756706587,-2.82106781187],[-1,0,0])
  ]
 },
 "BL": {
  "names": ["BL_A v1:1","BL_Coxa_motor:1","Servo_Mount v1 (2):1","rodend_5mm v2 (1):1","servo_arm_mod_L v1 (1):1","femur v1 (2):1","rodend_100mm v2:1","tibia_L v1 (1):1"],
  "foot": [10.6113056283,10.4758608852,-10.9966919484],
  "joints": [
   (2,"A","C",[6.71365352618,8.17636543571,4.25],[0,1,0]),
   (4,"H","A",[9.60865352595,11.73136543574,4.25],[-1,0,0]),
   (5,"M","A",[9.60865352595,9.73136543574,2.25],[1,0,0]),
   (6,"S","M",[9.86865352595,8.03430916090,3.94705627485],[1,0,0]),
   (7,"S","H",[9.88365352595,10.03430916090,5.94705627485],[1,0,0]),
   (8,"F","A",[9.60865352595,11.73136543574,4.25],[1,0,0]),
   (9,"R","H",[9.78365352595,13.64055374495,6.15918830920],[1,0,0]),
   (10,"T","F",[11.43365352595,18.80243324761,-2.82106781186],[1,0,0]),
   (11,"T","R",[10.33365352595,20.71162155681,-0.91187950266],[1,0,0])
  ]
 },
 "BR": {
  "names": ["BR_A v1:1","BR_Coxa_motor:1","Servo_Mount v1 (3):1","rodend_5mm v2 (2):1","servo_arm_mod_R v1 (1):1","femur v1 (3):1","rodend_100mm v2 (1):1","tibia_R v1 (1):1"],
  "foot": [-9.65900520256,10.8190064738,-11.3486660123],
  "joints": [
   (12,"A","C",[-5.78634658581,8.17636544319,4.25],[0,1,0]),
   (13,"F","A",[-8.68134658581,11.73136544319,4.25],[-1,0,0]),
   (14,"H","A",[-8.68134658581,11.73136544319,4.25],[-1,0,0]),
   (15,"M","A",[-8.68134658581,9.73136544319,2.25],[-1,0,0]),
   (16,"S","M",[-8.94134658581,8.03430916835,3.94705627485],[-1,0,0]),
   (17,"S","H",[-8.85634658581,10.03430916835,5.94705627485],[-1,0,0]),
   (18,"R","H",[-8.95634658581,13.64055375240,6.15918830920],[1,0,0]),
   (19,"T","F",[-10.50634658581,18.80243325506,-2.82106781186],[-1,0,0]),
   (20,"T","R",[-9.45634658581,20.71162156426,-0.91187950266],[-1,0,0])
  ]
 }
}

base = ["base_link:1","front v1:1","Raspberrypiwall:1","Back:1","Charger_wall v1:1"] + [leg["names"][1] for leg in LEGS.values()]
names = {"A":0,"C":1,"M":2,"S":3,"H":4,"F":5,"R":6,"T":7}
occurrences = [{"path":n,"component":n.rsplit(":",1)[0]} for n in base]
joints=[]
for _,leg in LEGS.items():
    for code in ("A","M","S","H","F","R","T"):
        path=leg["names"][names[code]]
        item={"path":path,"component":path.rsplit(":",1)[0]}
        if code=="T": item["preview_foot_proxy_cm"]=leg["foot"]
        occurrences.append(item)
    for number,a,b,p,axis in leg["joints"]:
        matrix=[1,0,0,p[0],0,1,0,p[1],0,0,1,p[2],0,0,0,1]
        joints.append({
            "name":f"rotate{number}","kind":"AsBuiltJoint",
            "occurrence_1":leg["names"][names[a]],
            "occurrence_2":leg["names"][names[b]],
            "is_suppressed":False,
            "motion_type":"adsk::fusion::RevoluteJointMotion",
            "rotation_axis_vector":axis,
            "as_built_transform_cm":matrix,
            "rotation_limits":{"minimum_enabled":False,"maximum_enabled":False}
        })
report={
 "provenance":"Manual transcription from the earlier Fusion audit; compact reference only, not a full CAD export.",
 "model_name":"assamble v1 - 36-joint reference",
 "unit_notes":"Fusion joint positions in cm, angles in radians",
 "summary":{"occurrences":len(occurrences),"joints":0,"as_built_joints":len(joints),"rigid_groups":1},
 "occurrences":occurrences,"joints":[],
 "as_built_joints":sorted(joints,key=lambda j:int(j["name"][6:])),
 "rigid_groups":[{"name":"rigid_grp","occurrences":base}]
}
out=Path(__file__).resolve().parent/"audit_excerpt_reference.json"
out.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding="utf-8")
print("Reference generated:",out,"| occurrences",len(occurrences),"| joints",len(joints))
assert len(occurrences)==37 and len(joints)==36
