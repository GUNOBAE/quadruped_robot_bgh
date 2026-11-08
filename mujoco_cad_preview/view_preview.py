"""Show the fixed-base schematic MuJoCo pose preview.

This uses mj_forward() on scripted joint coordinates. No mj_step() and
no reinforcement learning, standing, gravity, or physical walking.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from xml.etree import ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--verify-only", action="store_true")
    args = parser.parse_args()

    model_path = ROOT / "output" / "quadruped_preview.xml"
    pose_path = ROOT / "output" / "preview_motion.json"
    names_path = ROOT / "output" / "replay_joint_names.json"
    for path in (model_path, pose_path, names_path):
        if not path.is_file():
            raise FileNotFoundError(
                str(path) + " missing. Run NEW_PC_START.cmd first."
            )
    # Unicode-safe path read on Windows.
    model = mujoco.MjModel.from_xml_string(model_path.read_text("utf-8"))
    data = mujoco.MjData(model)
    motion = json.loads(pose_path.read_text(encoding="utf-8"))
    names = json.loads(names_path.read_text(encoding="utf-8"))
    addresses = []
    for name in names:
        joint_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, name
        )
        if joint_id < 0:
            raise RuntimeError("Missing joint: " + name)
        addresses.append(int(model.jnt_qposadr[joint_id]))

    site_pairs = []
    root = ET.parse(model_path).getroot()
    for e in root.findall("equality/connect"):
        a = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, e.get("site1")
        )
        b = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, e.get("site2")
        )
        if a < 0 or b < 0:
            raise RuntimeError("Loop closure site missing")
        site_pairs.append((a, b))
    maximum_gap = 0.0
    for f in motion:
        data.qpos[addresses] = np.asarray(f["q"], dtype=float)
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)
        for a, b in site_pairs:
            gap = float(np.linalg.norm(
                data.site_xpos[a] - data.site_xpos[b]
            ))
            maximum_gap = max(maximum_gap, gap)
    if not math.isfinite(maximum_gap) or maximum_gap > 1e-5:
        raise RuntimeError(
            f"Loop closure gap {maximum_gap:.6g}m exceeds tolerance"
        )
    report = {
        "mujoco_version": mujoco.__version__,
        "mjcf_compiled": True,
        "poses_verified": len(motion),
        "maximum_site_gap_m": maximum_gap,
        "nq": model.nq, "nv": model.nv, "nu": model.nu,
        "physics_stepping": False,
        "rl_training": False,
    }
    (ROOT / "output" / "local_mujoco_check.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )
    print(json.dumps(report, indent=2))
    if args.verify_only:
        return
    import mujoco.viewer
    print("STICK/LINKAGE PREVIEW ONLY. No real CAD meshes or walking.")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = [0.0, 0.015, -0.02]
        viewer.cam.distance = 0.65
        viewer.cam.azimuth = 125
        viewer.cam.elevation = -22
        start = time.perf_counter()
        while viewer.is_running():
            index = int(
                (time.perf_counter() - start) / .07
            ) % len(motion)
            with viewer.lock():
                data.qpos[addresses] = motion[index]["q"]
                data.qvel[:] = 0.0
                mujoco.mj_forward(model, data)
            viewer.sync()
            time.sleep(.01)


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        print("FAILED: send the error text.", file=sys.stderr)
        sys.exit(1)
