"""Evaluate saved command-conditioned PPO policy on 6 movement directions.

This tests simulated tracking and falls. It does NOT evaluate a real robot.
Use --view for interactive MuJoCo, --direction all for a text report.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import mujoco.viewer
import numpy as np
from stable_baselines3 import PPO

from rl_env import BGHCommandEnv, DIRECTIONS, angle_diff, ROOT


def run_direction(policy: PPO, direction: str, obs_mode: str,
                  seconds: float, view: bool, seed: int):
    env = BGHCommandEnv(
        stage="all", obs_mode=obs_mode, max_seconds=seconds,
        randomize_contact=False, command=direction
    )
    obs, _ = env.reset(seed=seed)
    origin = env.data.xpos[env.base_id].copy()
    R0 = env._rotation()
    yaw0 = env._yaw(R0)
    measures = []
    total_reward = 0.
    fell = False
    def rollout(viewer=None):
        nonlocal obs, total_reward, fell
        for _ in range(env.max_steps):
            action, _ = policy.predict(obs, deterministic=True)
            obs, reward, term, trunc, info = env.step(action)
            measures.append(info)
            total_reward += reward
            if viewer is not None:
                with viewer.lock():
                    viewer.sync()
                if not viewer.is_running():
                    break
                time.sleep(max(0., env.dt * .5))
            if term or trunc:
                fell = bool(info["fall"])
                break

    try:
        if view:
            with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
                viewer.cam.distance = .7
                viewer.cam.azimuth = 125
                viewer.cam.elevation = -18
                viewer.cam.lookat[:] = [0., 0., .08]
                rollout(viewer)
        else:
            rollout()
        delta_local = R0.T @ (env.data.xpos[env.base_id] - origin)
        measured_seconds = env.elapsed_steps * env.dt
        avg = np.mean([m["velocity_achieved"] for m in measures], axis=0) \
            if measures else np.zeros(3)
        desired = np.asarray(DIRECTIONS[direction])
        yaw_change = angle_diff(env._yaw(env._rotation()), yaw0)
        distances = {
            "forward_m": float(-delta_local[1]),
            "left_m": float(delta_local[0]),
            "yaw_change_rad": float(yaw_change),
        }
        errors = np.abs(avg - desired)
        candidate = bool(
            measured_seconds >= min(seconds, env.max_steps * env.dt) - env.dt
            and not fell and float(max(errors[:2])) < .04
            and float(errors[2]) < .23
        )
        return {
            "direction": direction,
            "desired_velocity": desired.tolist(),
            "achieved_mean_velocity": avg.tolist(),
            "absolute_velocity_error": errors.tolist(),
            "displacement": distances,
            "simulated_seconds": measured_seconds,
            "fell": fell,
            "mean_body_height_m": float(np.mean([m["body_height_m"]
                                                for m in measures])) if measures else 0.,
            "max_tilt_deg": max([m["tilt_deg"] for m in measures], default=0.),
            "mean_foot_contacts": float(np.mean([m["feet_in_contact"] for m in measures]))
                                  if measures else 0.,
            "cumulative_reward": float(total_reward),
            "preliminary_direction_tracking_candidate": candidate,
            "simulation_only": True,
        }
    finally:
        env.close()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--policy", type=Path,
                   default=ROOT / "output" / "rl_runs" / "all_sim_ppo.zip")
    p.add_argument("--direction", choices=[*DIRECTIONS, "all"], default="all")
    p.add_argument("--obs-mode", choices=("sim", "imu"), default="sim")
    p.add_argument("--seconds", type=float, default=6.)
    p.add_argument("--seed", type=int, default=123)
    p.add_argument("--view", action="store_true")
    a = p.parse_args()
    if not .3 <= a.seconds <= 30:
        p.error("--seconds must be 0.3..30")
    if not a.policy.is_file():
        raise FileNotFoundError(
            str(a.policy) + " missing. Run RUN_RL_TRAIN.cmd first."
        )
    if a.view and a.direction == "all":
        p.error("For GUI demo select ONE direction; use --direction all without --view for report")
    meta = a.policy.with_suffix(".json")
    if meta.is_file():
        saved = json.loads(meta.read_text(encoding="utf-8"))
        if saved.get("obs_mode") != a.obs_mode:
            raise RuntimeError("Policy observation mode and --obs-mode differ")
    policy = PPO.load(str(a.policy), device="cpu")
    test_directions = (
        ("forward", "backward", "left", "right", "turn_left", "turn_right")
        if a.direction == "all" else (a.direction,)
    )
    results = []
    for direction in test_directions:
        print("Evaluating:", direction, "SIMULATION ONLY")
        result = run_direction(
            policy, direction, a.obs_mode, a.seconds, a.view, a.seed
        )
        results.append(result)
        print(
            direction, "distance", result["displacement"],
            "fell", result["fell"],
            "mean_velocity", result["achieved_mean_velocity"],
            "candidate", result["preliminary_direction_tracking_candidate"]
        )
    report = {
        "policy": str(a.policy),
        "simulation_only": True,
        "real_hardware_tested": False,
        "caution": "Temporary physics, no verified motor mapping or silicone material calibration.",
        "results": results,
    }
    path = ROOT / "output" / "rl_runs" / "direction_evaluation.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2),
                    encoding="utf-8")
    print("DIRECTION REPORT:", path)
    print("A policy ZIP is not proof of successful walking; inspect measured velocity and falls.")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        sys.exit(1)
