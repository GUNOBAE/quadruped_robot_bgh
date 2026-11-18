"""BGH quadruped: VISIBLE repeated single-leg swing numerical IK (SIMULATION-ONLY).

BEFORE ANY GAIT/RL: demonstrate that one selected foot can be raised
and moved a few mm using the actual closed-loop MuJoCo assembly.

This is a FIXED-BASE, ZERO-GRAVITY, CONTACT-DISABLED experiment:
  - Do NOT interpret it as stable walking or load-bearing.
  - Use the existing 12 UNVERIFIED virtual actuator channels only.
  - It does not change CAD, Physics V1, RL PPO weights, or hardware.
  - Numerical IK is estimated from MuJoCo foot response to small
    +/- actuator setpoints; the closed 4-bar mechanisms are preserved.

Read: SWING_IK_GUIDE_KR.md (in the same GitHub branch).
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path
from xml.etree import ElementTree as ET

import mujoco
import mujoco.viewer
import numpy as np

from probe_actuators import build_fixed_base
from run_physics_v1 import load_model

HERE = Path(__file__).resolve().parent
OUT = HERE / "output"
LEGS = ("FL", "FR", "BL", "BR")


def xml_with_target(source: Path, destination: Path):
    """Copy Physics V1, lock torso and turn off gravity/contact (no new motors)."""
    toes, loops = build_fixed_base(source, destination)
    tree = ET.parse(destination)
    root = tree.getroot()
    root.set("model", "BGH_OneLeg_ClosedLoop_IK_FIXED_BASE_SIM_ONLY")
    world = root.find("worldbody")
    if world is None:
        raise ValueError("Missing worldbody")
    target = ET.SubElement(
        world, "body", name="swing_target_marker", mocap="true", pos="0 0 .15"
    )
    ET.SubElement(
        target, "geom", name="swing_goal",
        type="sphere", size=".004",
        rgba=".15 .95 .25 .85",
        contype="0", conaffinity="0", density="0", group="3"
    )
    ET.indent(root, space="  ")
    tree.write(destination, encoding="utf-8", xml_declaration=True)
    return toes, loops


def foot_sites(model):
    names = sorted(
        model.geom(i).name for i in range(model.ngeom)
        if (model.geom(i).name or "").startswith("foot_collision_")
    )
    if len(names) != 4:
        raise RuntimeError(f"Expected 4 foot colliders, found {names}")
    out = []
    for geom_name in names:
        site_name = geom_name.replace("foot_collision_", "foot_", 1)
        site_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, site_name
        )
        if site_id < 0:
            raise RuntimeError(f"No foot site for {geom_name}")
        out.append((site_name, site_id))
    return out


def settle(model, ctrl, seconds=.4):
    """Repeat independent zero/perturbed experiments under identical initial state."""
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    data.ctrl[:] = ctrl
    steps = max(1, int(round(seconds / model.opt.timestep)))
    for _ in range(steps):
        mujoco.mj_step(model, data)
    if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
        raise RuntimeError("Nonfinite state during fixed-base servo response test")
    return data


def equality_gap(data, site_pairs):
    return max(
        float(np.linalg.norm(data.site_xpos[a] - data.site_xpos[b]))
        for a, b in site_pairs
    )


def identify_foot(model, feet, actuator_idx, baseline, probe_rad):
    """Verify the leg coxa actuator moves a unique foot (not an assumed file order)."""
    ctrl = np.zeros(model.nu)
    ctrl[actuator_idx] = probe_rad
    changed = settle(model, ctrl, .4)
    movements = np.asarray([
        np.linalg.norm(changed.site_xpos[idx] - baseline.site_xpos[idx])
        for _, idx in feet
    ])
    winner = int(np.argmax(movements))
    other_max = float(np.max(np.delete(movements, winner)))
    if movements[winner] < .0005 or other_max > max(.001, .25 * movements[winner]):
        raise RuntimeError(
            f"Actuator-to-foot identification ambiguous: "
            f"movement (m) {movements.tolist()}"
        )
    return winner, movements.tolist()


def numerical_jacobian(model, foot_id, motor_indices, probe_rad):
    """J[:,j] ~= [foot(+eps)-foot(-eps)]/(2eps), in metres/radian.

    MuJoCo steps the actual closed-loop mechanism each time; we do not
    pretend the robot is three serial unconstrained hinge joints.
    """
    matrix = np.zeros((3, len(motor_indices)))
    for col, motor in enumerate(motor_indices):
        ctrl = np.zeros(model.nu)
        ctrl[motor] = probe_rad
        plus = settle(model, ctrl, .45).site_xpos[foot_id].copy()
        ctrl[motor] = -probe_rad
        minus = settle(model, ctrl, .45).site_xpos[foot_id].copy()
        matrix[:, col] = (plus - minus) / (2 * probe_rad)
    return matrix


def easing(s):
    return .5 - .5 * math.cos(math.pi * s)


def leg_swing(args):
    if args.leg not in LEGS:
        raise ValueError("Leg must be one of: FL, FR, BL, BR")
    OUT.mkdir(exist_ok=True)
    source = OUT / "physics_v1.xml"
    if not source.is_file():
        raise FileNotFoundError("Run RUN_PHYSICS_V1.cmd first: physics_v1.xml missing")
    fixed_path = OUT / "ik_swing_visible_v2_fixed.xml"
    toe_geom_names, loop_names = xml_with_target(source, fixed_path)
    model = load_model(fixed_path)
    if model.nu != 12:
        raise RuntimeError("12 provisional servo actuators expected")
    if np.any(model.opt.gravity != 0):
        raise RuntimeError("The IK bench must have gravity disabled")
    if model.nq != 28:
        raise RuntimeError("The IK bench should have a fixed torso, nq=28")
    feet = foot_sites(model)
    foot_ids = np.asarray([idx for _, idx in feet])
    pairs = []
    for one, two in loop_names:
        left = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, one)
        right = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, two)
        if left < 0 or right < 0:
            raise ValueError(f"Missing equality anchor pair: {one}, {two}")
        pairs.append((left, right))

    channels = [
        i for i in range(model.nu)
        if (model.actuator(i).name or "").startswith(args.leg + "_")
    ]
    if len(channels) != 3:
        raise RuntimeError(f"Expected 3 motors for {args.leg}, got {channels}")
    for i in channels:
        # Never exceed the already-compiled temporary MuJoCo control range.
        lo, hi = model.actuator_ctrlrange[i]
        if args.max_rad > min(abs(float(lo)), float(hi)):
            raise RuntimeError(
                f"Requested max-rad {args.max_rad:g} exceeds "
                f"MuJoCo temporary ctrl range of {model.actuator(i).name}: {lo:g}, {hi:g}"
            )
    coxa = [
        i for i in channels if model.actuator(i).name.endswith("_coxa")
    ]
    if len(coxa) != 1:
        raise RuntimeError(f"Could not identify {args.leg} coxa actuator")
    baseline = settle(model, np.zeros(model.nu), .45)
    target_foot_index, leg_response = identify_foot(
        model, feet, coxa[0], baseline, args.probe_rad
    )
    foot_name, foot_id = feet[target_foot_index]
    baseline_foot = baseline.site_xpos[foot_id].copy()
    baseline_others = baseline.site_xpos[foot_ids].copy()
    J = numerical_jacobian(model, foot_id, channels, args.probe_rad)
    singular = np.linalg.svd(J, compute_uv=False)
    condition = float(singular[0] / max(singular[-1], 1e-12))
    rank = int(np.linalg.matrix_rank(J, tol=.002))
    if rank < 3 or condition > 100:
        raise RuntimeError(
            f"Numerical IK Jacobian is singular/ill-conditioned: "
            f"rank={rank}, cond={condition:.2f}, J={J.tolist()}"
        )
    inverse = np.linalg.pinv(J, rcond=1e-3)
    print(f"Leg={args.leg} toe={foot_name}; motors="
          + ", ".join(model.actuator(i).name for i in channels))
    print("J = metres of foot movement per radian of actuator motion")
    for row in J:
        print(" ", "  ".join(f"{x:+.5f}" for x in row))
    print(f"Jacobian rank={rank}/3, condition number={condition:.2f}")

    data = settle(model, np.zeros(model.nu), .45)
    marker_bid = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "swing_target_marker"
    )
    mocap_id = int(model.body_mocapid[marker_bid]) if marker_bid >= 0 else -1
    if mocap_id < 0:
        raise RuntimeError("Target mocap marker missing")

    period = args.seconds
    count = max(25, int(round(period / .02)))
    ticks = max(1, int(round(.02 / model.opt.timestep)))
    control_prev = np.zeros(3)
    records = []
    initial_others = baseline_others.copy()
    max_gap = 0.
    viewer = None
    cycle = 0

    def run_one_frame(frame):
        nonlocal control_prev, max_gap
        s = frame / count
        # Cycle: SWING (lift + move forward), then RETURN at neutral height.
        # RETURN is only a visual stance proxy: gravity/contact remain OFF.
        if s <= .5:
            u = 2. * s
            lift = args.lift_mm / 1000 * math.sin(math.pi * u)
            forward = args.forward_mm / 1000 * easing(u)
            phase_name = "SWING"
        else:
            u = 2. * (s - .5)
            lift = 0.
            forward = args.forward_mm / 1000 * (1. - easing(u))
            phase_name = "RETURN_NO_CONTACT"
        desired = baseline_foot + np.array((0., -forward, lift))
        measured = data.site_xpos[foot_id].copy()
        delta = desired - baseline_foot
        correction = desired - measured
        q_des = inverse @ (delta + args.feedback * correction)
        q_des = np.clip(q_des, -args.max_rad, args.max_rad)
        q_new = control_prev + np.clip(
            q_des - control_prev, -args.rate_rad, args.rate_rad
        )
        control_prev = q_new
        if viewer is not None:
            with viewer.lock():
                data.ctrl[:] = 0.0
                data.ctrl[channels] = q_new
                data.mocap_pos[mocap_id] = desired
                for _ in range(ticks):
                    mujoco.mj_step(model, data)
        else:
            data.ctrl[:] = 0.0
            data.ctrl[channels] = q_new
            data.mocap_pos[mocap_id] = desired
            for _ in range(ticks):
                mujoco.mj_step(model, data)

        if not (np.all(np.isfinite(data.qpos)) and
                np.all(np.isfinite(data.qvel))):
            raise RuntimeError("Nonfinite state during visible IK swing")
        measured_after = data.site_xpos[foot_id].copy()
        gap = equality_gap(data, pairs)
        max_gap = max(max_gap, gap)
        if gap > .002:
            raise RuntimeError(
                "Closed-linkage anchor gap > 2mm. STOP and inspect geometry."
            )
        other_disp = np.linalg.norm(
            (data.site_xpos[foot_ids] - initial_others)[
                np.arange(4) != target_foot_index
            ], axis=1
        ).max()
        force_clip = []
        for i in channels:
            limit = max(abs(float(x)) for x in model.actuator_forcerange[i])
            if limit > 1e-8:
                force_clip.append(abs(float(data.actuator_force[i])) / limit)
        rec = {
            "time_s": float(data.time),
            "stage": s,
            "phase": phase_name,
            "goal_x_m": float(desired[0]),
            "goal_y_m": float(desired[1]),
            "goal_z_m": float(desired[2]),
            "actual_x_m": float(measured_after[0]),
            "actual_y_m": float(measured_after[1]),
            "actual_z_m": float(measured_after[2]),
            "target_control_radians": [float(x) for x in q_new],
            "foot_error_mm": float(1000*np.linalg.norm(measured_after - desired)),
            "other_foot_max_drift_mm": float(1000 * other_disp),
            "loop_gap_mm": float(1000 * gap),
            "force_limit_fraction_max": max(force_clip, default=0.0),
        }
        # Keep only FIRST full cycle for independent and comparable metrics.
        if cycle == 0:
            records.append(rec)

    try:
        if args.view:
            viewer = mujoco.viewer.launch_passive(model, data)
            viewer.cam.lookat[:] = baseline_foot + np.array([0., -.006, .012])
            viewer.cam.distance = .22
            viewer.cam.azimuth = 125
            viewer.cam.elevation = -15
            print("VIEWER: zoom in on chosen leg; GREEN ball = target.")
            print("Repeating large SWING and virtual RETURN until you close the window.")
            print("Gravity OFF / contact OFF / body FIXED. No stance/real walking yet.")
        while True:
            if viewer is None and cycle >= 1:
                break
            if viewer is not None and not viewer.is_running():
                break
            if args.cycles > 0 and cycle >= args.cycles:
                break

            if cycle > 0:
                # Ensure each visual replay starts from identical initial state
                # rather than accumulating error or drifting across cycles.
                if viewer is not None:
                    with viewer.lock():
                        mujoco.mj_resetData(model, data)
                        data.ctrl[:] = 0.
                        for _ in range(int(.45/model.opt.timestep)):
                            mujoco.mj_step(model, data)
                else:
                    mujoco.mj_resetData(model, data)
                    data.ctrl[:] = 0.
                    for _ in range(int(.45/model.opt.timestep)):
                        mujoco.mj_step(model, data)
                control_prev[:] = 0.0

            for frame in range(count + 1):
                if viewer is not None and not viewer.is_running():
                    break
                run_one_frame(frame)
                if viewer is not None:
                    viewer.sync()
                    time.sleep(.02)
            cycle += 1
            if cycle == 1:
                print("First full swing cycle completed; recording fixed for analysis.")
        # Finite cycles: allow the user to inspect the final pose without
        # the MuJoCo window disappearing.
        if viewer is not None and args.cycles > 0 and viewer.is_running():
            print("Cycles complete. Close MuJoCo window to write report.")
            while viewer.is_running():
                viewer.sync()
                time.sleep(.04)
    finally:
        if viewer is not None:
            viewer.close()

    if len(records) < 2:
        raise RuntimeError("Swing experiment did not run enough frames")
    lift_achieved = 1000 * max(
        r["actual_z_m"] - baseline_foot[2] for r in records
    )
    target_lift = args.lift_mm
    peak_forward = -1000 * min(
        r["actual_y_m"] - baseline_foot[1] for r in records
    )
    return_error = 1000 * np.linalg.norm(np.array([
        records[-1]["actual_x_m"],
        records[-1]["actual_y_m"],
        records[-1]["actual_z_m"],
    ]) - baseline_foot)
    peak_error = max(r["foot_error_mm"] for r in records)
    max_other_drift = max(r["other_foot_max_drift_mm"] for r in records)
    peak_force = max(r["force_limit_fraction_max"] for r in records)
    if len(records) < count + 1:
        status = "INCOMPLETE_VIEWER_CLOSED"
    elif (lift_achieved >= .7*target_lift and peak_forward >= .7*args.forward_mm
          and return_error < 4.0 and peak_error < 10.
          and max_other_drift < 3.0 and max_gap < .001):
        status = "VISIBLE_SWING_KINEMATIC_CANDIDATE"
    else:
        status = "SWING_NOT_YET_RELIABLE"
    summary = {
        "status": status,
        "experiment": "fixed-base zero-gravity numerical Jacobian IK",
        "simulation_only": True,
        "not_a_standing_or_walking_test": True,
        "leg": args.leg,
        "toe_site": foot_name,
        "motor_names": [model.actuator(i).name for i in channels],
        "jacobian_m_per_rad": J.tolist(),
        "jacobian_rank": rank,
        "jacobian_condition": condition,
        "coxa_foot_mapping_response_m": leg_response,
        "goal_lift_mm": target_lift,
        "achieved_peak_lift_mm": float(lift_achieved),
        "goal_forward_mm": args.forward_mm,
        "achieved_peak_forward_mm": float(peak_forward),
        "end_return_error_mm": float(return_error),
        "completed_viewer_cycles": cycle,
        "viewer_demo_repeats": bool(args.view),
        "max_position_error_mm": float(peak_error),
        "max_other_feet_drift_mm": float(max_other_drift),
        "max_loop_gap_mm": float(1000*max_gap),
        "max_force_fraction_of_provisional_limit": float(peak_force),
        "frames": len(records),
        "warnings": [
            "Fixed torso, gravity OFF, foot contact OFF",
            "Servo mapping is only virtual; no real hardware is controlled",
            "Numerical finite-difference IK is a local approximation",
            "A closed-loop mechanical constraint mismatch should be investigated before advancing",
            "The green marker is the desired toe site, not a contact patch",
        ],
    }
    report_path = OUT / "ik_swing_visible_v2_report.json"
    csv_path = OUT / "ik_swing_visible_v2_trajectory.csv"
    report_path.write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    cols = [
        "time_s", "stage", "phase",
        "goal_x_m", "goal_y_m", "goal_z_m",
        "actual_x_m", "actual_y_m", "actual_z_m",
        "foot_error_mm", "other_foot_max_drift_mm",
        "loop_gap_mm", "force_limit_fraction_max",
        "target_control_radians"
    ]
    with csv_path.open("w", newline="", encoding="utf-8-sig") as fh:
        writer = csv.DictWriter(fh, fieldnames=cols)
        writer.writeheader()
        for rec in records:
            writer.writerow({
                **rec,
                "target_control_radians": "|".join(
                    f"{x:.6g}" for x in rec["target_control_radians"]
                )
            })
    print("--- SWING IK RESULT (no gravity / no contact) ---")
    print("Status:", status)
    print(f"Peak foot lift: {lift_achieved:.2f}mm / target {args.lift_mm:.2f}mm")
    print(f"Peak forward: {peak_forward:.2f}mm / target {args.forward_mm:.2f}mm")
    print(f"Foot return-to-start error: {return_error:.2f}mm")
    print(f"Max toe tracking error: {peak_error:.2f}mm")
    print(f"Max other foot drift: {max_other_drift:.3f}mm")
    print(f"Max loop anchor gap: {max_gap*1000:.4f}mm")
    print("Report:", report_path)
    print("Trajectory:", csv_path)
    print("NOT STANDING / NOT WALKING / NOT YET SIM2REAL.")
    return summary


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--leg", choices=LEGS, default="FL")
    ap.add_argument("--view", action="store_true")
    ap.add_argument("--lift-mm", type=float, default=22.)
    ap.add_argument("--forward-mm", type=float, default=20.)
    ap.add_argument("--seconds", type=float, default=3.)
    ap.add_argument("--cycles", type=int, default=0,
                    help="Viewer cycles. 0 repeats until window is closed.")
    ap.add_argument("--probe-rad", type=float, default=.07)
    ap.add_argument("--max-rad", type=float, default=.235)
    ap.add_argument("--rate-rad", type=float, default=.03)
    ap.add_argument("--feedback", type=float, default=.45)
    a = ap.parse_args()
    if not (5 <= a.lift_mm <= 30 and 5 <= a.forward_mm <= 30):
        ap.error("Visible swing target must be 5..30mm (kinematic-only)")
    if not (1 <= a.seconds <= 8 and .02 <= a.probe_rad <= .10):
        ap.error("Invalid swing duration/probe size")
    if not (.06 <= a.max_rad <= .24 and .01 <= a.rate_rad <= .05):
        ap.error("Invalid local virtual servo limits")
    if a.cycles < 0 or a.cycles > 100:
        ap.error("--cycles must be 0 (repeat until closed) or 1..100")
    if not 0 <= a.feedback <= 1:
        ap.error("feedback must be 0 to 1")
    leg_swing(a)


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        sys.exit(1)
