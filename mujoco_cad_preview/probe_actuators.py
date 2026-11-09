"""Diagnose whether 12 provisional MuJoCo actuators actually move the feet.

This is a NON-LEARNING, ZERO-GRAVITY, FIXED-BASE bench test made from
the existing Physics V1 model. It does not modify your original XML,
trained policies, Fusion source or robot hardware.

IMPORTANT: no inference from actuator names to actual servo channels.
The 12 motors are PROVISIONAL until the real servo linkage is verified.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

from run_physics_v1 import load_model

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "output"
RUNS = OUT / "rl_runs"


def build_fixed_base(source: Path, destination: Path) -> tuple[list, list]:
    tree = ET.parse(source)
    root = tree.getroot()
    if len(root.findall(".//freejoint[@name='base_free']")) != 1:
        raise RuntimeError("Expected one root freejoint called base_free")
    body = next(b for b in root.findall(".//body")
                if b.find("freejoint[@name='base_free']") is not None)
    body.remove(body.find("freejoint[@name='base_free']"))
    option = root.find("option")
    if option is None:
        raise RuntimeError("Missing simulation options")
    option.set("gravity", "0 0 0")
    flag = option.find("flag")
    if flag is None:
        flag = ET.SubElement(option, "flag")
    flag.set("contact", "disable")
    root.set("model", "BGH_FIXED_BASE_ACTUATOR_DIAGNOSTIC_ONLY")
    toes = [
        name for name in (
            g.get("name") for g in root.findall(".//geom")
        ) if name and name.startswith("foot_collision_")
    ]
    loops = [
        (e.get("site1"), e.get("site2"))
        for e in root.findall("./equality/connect")
    ]
    if len(toes) != 4 or len(loops) != 16:
        raise RuntimeError("Unexpected foot contact / closed-loop structure")
    destination.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space="  ")
    tree.write(destination, encoding="utf-8", xml_declaration=True)
    return toes, loops


def simulation_probe(model, foot_site_ids, loop_site_pairs, idx, value,
                     settle_seconds, run_seconds):
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    data.ctrl[:] = 0
    base_steps = max(5, round(settle_seconds / model.opt.timestep))
    for _ in range(base_steps):
        mujoco.mj_step(model, data)
    foot_initial = data.site_xpos[foot_site_ids].copy()
    joint_id = int(model.actuator_trnid[idx, 0])
    adr = int(model.jnt_qposadr[joint_id])
    initial_joint = float(data.qpos[adr])
    data.ctrl[idx] = float(value)
    forces = []
    error_trace = []
    total_steps = max(5, round(run_seconds / model.opt.timestep))
    for i in range(total_steps):
        mujoco.mj_step(model, data)
        if i % 5 == 0:
            forces.append(float(data.actuator_force[idx]))
            error_trace.append(abs(float(value) - float(data.qpos[adr])))
        if not (np.all(np.isfinite(data.qpos)) and
                np.all(np.isfinite(data.qvel))):
            raise RuntimeError("Nonfinite simulation state in actuator probe")
    foot_delta = data.site_xpos[foot_site_ids] - foot_initial
    max_gap = 0.
    for a, b in loop_site_pairs:
        max_gap = max(max_gap,
                      float(np.linalg.norm(data.site_xpos[a] -
                                           data.site_xpos[b])))
    force_limit = model.actuator_forcerange[idx].copy()
    clip_count = sum(
        f >= float(force_limit[1]) - 1e-4
        or f <= float(force_limit[0]) + 1e-4 for f in forces
    )
    return {
        "control_target_rad": float(value),
        "joint_change_deg": math.degrees(float(data.qpos[adr]) - initial_joint),
        "final_tracking_error_deg": math.degrees(abs(float(value)-float(data.qpos[adr]))),
        "mean_tracking_error_deg": math.degrees(float(np.mean(error_trace))),
        "force_saturation_fraction": clip_count / max(1, len(forces)),
        "max_constraint_gap_m": max_gap,
        "foot_displacements_m": foot_delta.tolist(),
        "foot_displacement_norm_cm": (100 * np.linalg.norm(foot_delta, axis=1)).tolist(),
        "foot_vertical_displacement_cm": (100 * foot_delta[:, 2]).tolist(),
        "max_foot_displacement_cm": 100 * float(np.linalg.norm(foot_delta, axis=1).max()),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=OUT / "physics_rl_v0.xml")
    parser.add_argument("--pulse-rad", type=float, default=.12)
    parser.add_argument("--settle-s", type=float, default=.15)
    parser.add_argument("--pulse-s", type=float, default=.3)
    args = parser.parse_args()
    if not (.02 <= args.pulse_rad <= .18):
        parser.error("--pulse-rad should be .02.. .18")
    if not (.04 <= args.settle_s <= 3 and .05 <= args.pulse_s <= 3):
        parser.error("settle-s or pulse-s out of range")
    if not args.source.is_file():
        raise FileNotFoundError("Run build_rl_model.py first")
    RUNS.mkdir(parents=True, exist_ok=True)
    filename = RUNS / "actuator_probe_fixed_base.xml"
    foot_names, loop_names = build_fixed_base(args.source, filename)
    model = load_model(filename)
    if model.nu != 12:
        raise RuntimeError("Expected 12 active virtual position actuators")
    foot_site_ids = []
    for geom in sorted(foot_names):
        name = geom.replace("foot_collision_", "foot_")
        sid = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, name)
        if sid < 0:
            raise RuntimeError("Missing foot site " + name)
        foot_site_ids.append(sid)
    loop_site_pairs = []
    for left, right in loop_names:
        a = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, left)
        b = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, right)
        if min(a, b) < 0:
            raise RuntimeError("Missing equality loop anchor")
        loop_site_pairs.append((a, b))

    report = {
        "experiment": "12 independent actuator pulses, FIXED BASE and ZERO GRAVITY",
        "simulation_only": True,
        "no_real_robot_control": True,
        "input_model": str(args.source),
        "foot_sites": [model.site(s).name for s in foot_site_ids],
        "pulse_radians": float(args.pulse_rad),
        "warning": ("Designed to test actuator-foot kinematic coupling, "
                    "NOT standing capability or actual servo channel mapping"),
        "results": [],
    }
    for idx in range(model.nu):
        j = int(model.actuator_trnid[idx, 0])
        record = {
            "actuator_index": idx,
            "actuator": model.actuator(idx).name,
            "driven_joint": model.joint(j).name,
            "joint_axis_local": model.jnt_axis[j].tolist(),
            "max_ctrlrange_rad": model.actuator_ctrlrange[idx].tolist(),
            "force_limit_Nm_proxy": model.actuator_forcerange[idx].tolist(),
        }
        for sign, key in ((1, "positive"), (-1, "negative")):
            record[key] = simulation_probe(
                model, foot_site_ids, loop_site_pairs, idx,
                sign * args.pulse_rad, args.settle_s, args.pulse_s
            )
        moved = max(record["positive"]["max_foot_displacement_cm"],
                    record["negative"]["max_foot_displacement_cm"])
        record["max_foot_effect_cm"] = moved
        record["small_foot_response_warning"] = moved < .1
        report["results"].append(record)
        print(
            f"[{idx:02d}] {record['actuator']:>16s} -> {record['driven_joint']:>9s}: "
            f"max toe motion={moved:.2f}cm, "
            f"joint response (+)={record['positive']['joint_change_deg']:+.2f}deg, "
            f"force-clipping (+)={record['positive']['force_saturation_fraction']:.0%}"
        )
    path = RUNS / "actuator_probe.json"
    path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print("ACTUATOR REPORT:", path)
    print("Do not tune PPO rewards before checking whether every intended servo moves its expected leg.")
    print("This probe tests model coupling, NOT real hardware mapping.")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        sys.exit(1)
