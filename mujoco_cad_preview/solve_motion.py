"""Compute a small, closure-consistent 97-frame joint PREVIEW (not walking).

SciPy least-squares solves passive hinges for a tiny scripted motion.
No physics stepping, gravity, friction, foot contacts, torque limits or RL.
Outputs files expected by view_preview.py / view_cad_preview.py.
"""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from build_preview import build_graph

ROOT = Path(__file__).resolve().parent
audit = json.loads((ROOT / "audit_excerpt_reference.json").read_text(encoding="utf-8"))
g = build_graph(audit)

origins = {k: np.asarray(v, dtype=float) for k, v in g["origins"].items()}
axes = [np.asarray(t["axis"], dtype=float) for t in g["tree"]]
driven = [d["tree_index"] for d in g["drives"]]
passive = [i for i in range(len(g["tree"])) if i not in driven]


def forward_kinematics(q):
    result = {g["root"]: (np.eye(3), np.zeros(3))}
    for i, t in enumerate(g["tree"]):
        R, p = result[t["parent"]]
        Rjoint = Rotation.from_rotvec(axes[i] * q[i]).as_matrix()
        result[t["child"]] = (
            R @ Rjoint,
            p + R @ (origins[t["child"]] - origins[t["parent"]]),
        )
    return result


def point(transform, body, p):
    R, origin = transform[body]
    return origin + R @ (np.asarray(p, dtype=float) - origins[body])


def residual(q):
    tf = forward_kinematics(q)
    out = []
    for edge in g["loops"]:
        for offset in (0.0, 0.01):
            p = np.asarray(edge["p"]) + np.asarray(edge["axis"]) * offset
            out.extend(point(tf, edge["a"], p) - point(tf, edge["b"], p))
    return np.asarray(out, dtype=float)


def scripted_targets(phase):
    values = []
    for drive in g["drives"]:
        leg, role = drive["name"].split("_", 1)
        shift = {"FL": 0, "FR": math.pi/2, "BL": math.pi, "BR": 3*math.pi/2}[leg]
        amplitude = {"coxa": .03, "femur_test": .06, "mount_test": .045}[role]
        values.append(amplitude * math.sin(phase + shift) * math.sin(phase/2)**2)
    return np.asarray(values)


def main():
    output = ROOT / "output"
    output.mkdir(parents=True, exist_ok=True)
    q = np.zeros(len(g["tree"]))
    h = 1e-6
    J = np.column_stack([
        (residual(q + np.eye(len(q))[i] * h) -
         residual(q - np.eye(len(q))[i] * h)) / (2 * h)
        for i in range(len(q))
    ])
    singular = np.linalg.svd(J, compute_uv=False)
    rank = int(sum(singular > 1e-7))
    frames = []
    errors = []
    evaluations = []
    begin = time.time()

    for f in range(97):
        phase = 2 * math.pi * f / 96
        q[driven] = scripted_targets(phase)

        def solve_fun(x):
            test = q.copy()
            test[passive] = x
            return residual(test)

        solution = least_squares(
            solve_fun, q[passive],
            xtol=1e-12, ftol=1e-12, gtol=1e-12, max_nfev=100
        )
        q[passive] = solution.x
        tf = forward_kinematics(q)
        error = float(np.linalg.norm(
            residual(q).reshape(-1, 3), axis=1
        ).max())
        if not solution.success or error > 1e-6:
            raise RuntimeError(f"Loop closure failed at frame {f}, gap={error}")
        errors.append(error)
        evaluations.append(solution.nfev)
        frames.append({"q": q.tolist()})

    # The MuJoCo joint ordering in our MJCF is the tree ordering.
    replay_names = [t["name"] for t in g["tree"]]
    (output / "preview_motion.json").write_text(
        json.dumps(frames, separators=(",", ":")), encoding="utf-8"
    )
    (output / "replay_joint_names.json").write_text(
        json.dumps(replay_names, indent=2), encoding="utf-8"
    )
    report = {
        "validation_type": "Offline kinematic loop-closure calculation, not MuJoCo dynamics",
        "frames": len(frames),
        "bodies": len(g["nodes"]),
        "tree_joints": len(g["tree"]),
        "mechanical_loops": len(g["loops"]),
        "constraint_jacobian_rank_at_zero_pose": rank,
        "expected_mobility_fixed_base_at_zero_pose": len(g["tree"]) - rank,
        "test_control_inputs": len(g["drives"]),
        "max_loop_anchor_gap_m": max(errors),
        "largest_solver_nfev": max(evaluations),
        "elapsed_seconds": time.time() - begin,
        "not_validated": [
            "Dynamic stepping", "Actual motor/servo mapping", "Physical mass and inertia",
            "Gravity", "Foot contact", "Walking gait", "Reinforcement learning"
        ],
    }
    (output / "kinematic_validation.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )
    print(
        f"Motion generated: {len(frames)} frames; "
        f"max closure gap={max(errors):.3g}m; rank={rank}/28"
    )


if __name__ == "__main__":
    main()
