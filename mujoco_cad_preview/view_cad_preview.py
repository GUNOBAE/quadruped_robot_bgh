"""View the 37 Fusion CAD meshes attached to the BGH MuJoCo linkage preview.

This only replays precomputed kinematic poses; it DOES NOT perform
physics stepping, servo control, reinforcement learning or walking.
It reads XML and STL bytes in Python to avoid Unicode Windows path failures.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from xml.etree import ElementTree as ET

import mujoco
import mujoco.viewer
import numpy as np

ROOT = Path(__file__).resolve().parent


def load_model(xml_file: Path):
    raw_xml = xml_file.read_text(encoding="utf-8")
    tree = ET.fromstring(raw_xml)
    mesh_entries = tree.findall("asset/mesh")
    if len(mesh_entries) != 37:
        raise RuntimeError(
            f"Expected 37 actual CAD mesh assets, found {len(mesh_entries)}"
        )
    # MuJoCo 3.17 supports this virtual file system (VFS).
    # Bytes are read using Python, so Korean path names work on Windows.
    with mujoco.MjVfs() as vfs:
        for entry in mesh_entries:
            fname = entry.attrib["file"]
            path = ROOT / "output" / "cad_meshes" / fname
            if not path.is_file():
                raise FileNotFoundError("Missing CAD STL: " + str(path))
            vfs[fname] = path.read_bytes()
        model = mujoco.MjModel.from_xml_string(raw_xml, vfs=vfs)
    return model


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--verify-only",
        action="store_true",
        help="Compile the CAD MJCF and check all 97 pose frames, no GUI",
    )
    args = parser.parse_args()
    model_path = ROOT / "output" / "cad_preview.xml"
    report_path = ROOT / "output" / "cad_fit_report.json"
    if not model_path.is_file():
        raise FileNotFoundError(
            "First run attach_cad_meshes.py; missing " + str(model_path)
        )
    report = json.loads(report_path.read_text(encoding="utf-8"))
    if report.get("errors") or report.get("converted") != 37:
        raise RuntimeError("CAD alignment failed: read output/cad_fit_report.json")

    model = load_model(model_path)
    data = mujoco.MjData(model)
    motion = json.loads(
        (ROOT / "output" / "preview_motion.json").read_text(encoding="utf-8")
    )
    names = json.loads(
        (ROOT / "output" / "replay_joint_names.json").read_text(encoding="utf-8")
    )
    addresses = []
    for name in names:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise RuntimeError("Missing replay joint in model: " + name)
        addresses.append(int(model.jnt_qposadr[joint_id]))
    print(
        f"MuJoCo {mujoco.__version__}: CAD mesh assets=37, "
        f"frames={len(motion)}, replay joints={len(names)}"
    )
    if not motion:
        raise ValueError("No preview frames found")
    # Do one complete pose evaluation before opening the viewer.
    for frame in motion:
        data.qpos[addresses] = np.asarray(frame["q"], dtype=float)
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)
        if not np.all(np.isfinite(data.qpos)):
            raise RuntimeError("Non-finite joint state")
    print("Kinematic replay loaded. NOT standing or walking physics.")
    if args.verify_only:
        return

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = [0.0, 0.0, 0.0]
        viewer.cam.distance = 0.65
        viewer.cam.azimuth = 125
        viewer.cam.elevation = -22
        start = time.perf_counter()
        while viewer.is_running():
            idx = int((time.perf_counter() - start) / 0.07) % len(motion)
            with viewer.lock():
                data.qpos[addresses] = motion[idx]["q"]
                data.qvel[:] = 0.0
                mujoco.mj_forward(model, data)
            viewer.sync()
            time.sleep(0.015)


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        import traceback
        print("CAD VIEWER FAILED:", error, file=sys.stderr)
        traceback.print_exc()
        sys.exit(1)
