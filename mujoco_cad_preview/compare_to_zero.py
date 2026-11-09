"""Compare TRAINED PPO against zero-action baseline in the exact same MuJoCo.

This is an educational diagnostic, not training, and NEVER sends real servo
commands. It does not overwrite training checkpoints or the original CAD.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
from stable_baselines3 import PPO

from rl_env import BGHCommandEnv, DIRECTIONS, angle_diff, ROOT


def trial(policy, direction, seconds, seed, mode):
    env = BGHCommandEnv(
        stage="all", obs_mode=mode, max_seconds=seconds,
        randomize_contact=False, command=direction
    )
    try:
        obs, _ = env.reset(seed=seed)
        start = env.data.xpos[env.base_id].copy()
        start_R = env._rotation()
        start_yaw = env._yaw(start_R)
        initial_height = float(env.data.qpos[2])
        initial_contact = int(env.last_contact.sum())
        commands, results = [], []
        previous = np.zeros(12, dtype=np.float64)
        for _ in range(env.max_steps):
            if policy is None:
                action = np.zeros(12, dtype=np.float32)
            else:
                action, _ = policy.predict(obs, deterministic=True)
            action = np.asarray(action, dtype=np.float64)
            obs, reward, terminated, truncated, info = env.step(action)
            results.append((float(reward), dict(info)))
            actual_joint = env.data.qpos[env.joint_qadr]
            commanded_joint = env.data.ctrl
            force_limit = np.maximum(
                np.abs(env.model.actuator_forcerange[:, 0]),
                np.abs(env.model.actuator_forcerange[:, 1])
            )
            valid_limit = force_limit > 1e-6
            torque_sat = np.mean(
                np.abs(env.data.actuator_force[valid_limit])
                >= .95 * force_limit[valid_limit]
            ) if np.any(valid_limit) else 0.
            toe_clearance = np.maximum(
                0., env.data.site_xpos[env.foot_site_ids, 2]
                - env.model.geom_size[env.feet_geom_ids, 0]
            )
            commands.append({
                "mean_abs_action": float(np.mean(np.abs(action))),
                "mean_abs_change": float(np.mean(np.abs(action - previous))),
                "action_saturation_fraction": float(np.mean(np.abs(action) >= .9)),
                "mean_servo_tracking_error_deg": float(
                    np.degrees(np.mean(np.abs(commanded_joint - actual_joint)))
                ),
                "force_limit_fraction": float(torque_sat),
                "feet_clearing_15mm": float(np.mean(toe_clearance >= .015)),
                "max_toe_clearance_cm": float(100 * np.max(toe_clearance)),
            })
            previous = action
            if terminated or truncated:
                break
        if not results:
            raise RuntimeError("The environment produced no simulation steps")
        R = env._rotation()
        delta_local = start_R.T @ (env.data.xpos[env.base_id] - start)
        avg = np.mean([i["velocity_achieved"] for _, i in results], axis=0)
        cmd = np.asarray(DIRECTIONS[direction])
        last = results[-1][1]
        reward_parts = {}
        for _, item in results:
            for key, value in item.get("reward_components", {}).items():
                reward_parts[key] = reward_parts.get(key, 0.) + float(value)
        return {
            "duration_s": len(results) * env.dt,
            "target_velocity_forward_left_yaw": cmd.tolist(),
            "mean_velocity_forward_left_yaw": avg.tolist(),
            "absolute_velocity_error": np.abs(avg - cmd).tolist(),
            "body_relative_displacement": {
                "forward_m": float(-delta_local[1]),
                "left_m": float(delta_local[0]),
                "yaw_rad": float(angle_diff(env._yaw(R), start_yaw)),
            },
            "total_reward": float(sum(r for r, _ in results)),
            "reward_components_total": reward_parts,
            "reward_components_per_step": {
                key: val / len(results) for key, val in reward_parts.items()
            },
            "initial_base_height_m": initial_height,
            "initial_foot_contacts": initial_contact,
            "end_base_height_m": float(last["body_height_m"]),
            "max_tilt_deg": float(max(i["tilt_deg"] for _, i in results)),
            "mean_foot_contacts": float(np.mean(
                [i["feet_in_contact"] for _, i in results]
            )),
            "mean_slip_proxy": float(np.mean(
                [i["contact_slip_proxy"] for _, i in results]
            )),
            "mean_abs_action": float(np.mean(
                [v["mean_abs_action"] for v in commands]
            )),
            "mean_abs_change": float(np.mean(
                [v["mean_abs_change"] for v in commands]
            )),
            "action_saturation_fraction": float(np.mean(
                [v["action_saturation_fraction"] for v in commands]
            )),
            "mean_servo_tracking_error_deg": float(np.mean(
                [v["mean_servo_tracking_error_deg"] for v in commands]
            )),
            "force_limit_fraction": float(np.mean(
                [v["force_limit_fraction"] for v in commands]
            )),
            "fraction_feet_15mm_above_floor": float(np.mean(
                [v["feet_clearing_15mm"] for v in commands]
            )),
            "max_toe_clearance_cm": float(max(
                v["max_toe_clearance_cm"] for v in commands
            )),
            "fell": bool(last["fall"]),
        }
    finally:
        env.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--policy", type=Path,
                        default=ROOT / "output/rl_runs/all_sim_ppo.zip")
    parser.add_argument("--obs-mode", choices=("sim", "imu"), default="sim")
    parser.add_argument("--seconds", type=float, default=6.0)
    parser.add_argument("--seed", type=int, default=123)
    parser.add_argument("--direction",
                        choices=["all", *DIRECTIONS.keys()], default="all")
    a = parser.parse_args()
    if not (.5 <= a.seconds <= 30):
        parser.error("--seconds should be in the range 0.5 to 30")
    if not a.policy.is_file():
        raise FileNotFoundError("Trained PPO policy not found: " + str(a.policy))
    policy = PPO.load(str(a.policy), device="cpu")
    metadata = a.policy.with_suffix(".json")
    if metadata.is_file():
        d = json.loads(metadata.read_text(encoding="utf-8"))
        if d.get("obs_mode") not in (None, a.obs_mode):
            raise RuntimeError("Policy observation mode differs from evaluation mode")
    directions = (
        ["stop", "forward", "backward", "left", "right", "turn_left", "turn_right"]
        if a.direction == "all" else [a.direction]
    )
    rows = []
    print("Comparing an unchanged zero-action baseline with trained PPO.")
    print("Both runs share identical initial state, command and contact conditions.")
    print("SIMULATION ONLY. No hardware calibration has been performed.")
    for direction in directions:
        baseline = trial(None, direction, a.seconds, a.seed, a.obs_mode)
        trained = trial(policy, direction, a.seconds, a.seed, a.obs_mode)
        reward_gain = trained["total_reward"] - baseline["total_reward"]
        errors_baseline = np.asarray(baseline["absolute_velocity_error"])
        errors_trained = np.asarray(trained["absolute_velocity_error"])
        if direction.startswith("turn_"):
            primary_axis = 2
        elif direction in ("left", "right"):
            primary_axis = 1
        else:
            primary_axis = 0
        tracking_gain = float(errors_baseline[primary_axis] -
                              errors_trained[primary_axis])
        command_scale = abs(float(DIRECTIONS[direction][primary_axis]))
        normalized_gain = (tracking_gain / max(command_scale, 1e-9)
                           if command_scale else 0.)
        crouch_vs_zero_m = (baseline["end_base_height_m"] -
                            trained["end_base_height_m"])
        # A small velocity improvement is not enough to claim useful walking.
        # Stand/stop is treated separately from directional gait.
        if direction == "stop":
            candidate = bool(
                not trained["fell"]
                and trained["max_tilt_deg"] < 10.
                and abs(trained["body_relative_displacement"]["forward_m"]) < .01
                and abs(trained["body_relative_displacement"]["left_m"]) < .01
            )
        else:
            candidate = bool(
                not trained["fell"]
                and normalized_gain >= .50
                and crouch_vs_zero_m < .01
                and trained["max_tilt_deg"] < 20.
                and trained["mean_foot_contacts"] >= 2.0
            )
        rows.append({
            "direction": direction,
            "zero_action_baseline": baseline,
            "trained_policy": trained,
            "total_reward_gain": float(reward_gain),
            "primary_axis_velocity_error_reduction": tracking_gain,
            "improved_primary_axis_tracking": tracking_gain > 0,
            "normalized_primary_axis_gain": float(normalized_gain),
            "crouch_vs_zero_m": float(crouch_vs_zero_m),
            "cautious_command_tracking_candidate": candidate,
            "warning": (
                "Previous policy_better_than_doing_nothing was an overly "
                "permissive single-axis metric. This diagnostic checks "
                "body height, tilt and normalized target tracking too."
            ),
        })
        print(
            f"{direction:>11}: base error={errors_baseline[primary_axis]:.4f} "
            f"ppo error={errors_trained[primary_axis]:.4f} "
            f"improvement={tracking_gain:+.4f} | "
            f"base reward={baseline['total_reward']:.1f} "
            f"ppo reward={trained['total_reward']:.1f} | "
            f"fall={trained['fell']} "
            f"ppo action={trained['mean_abs_action']:.3f} "
            f"action_sat={trained['action_saturation_fraction']:.0%} "
            f"servo_err={trained['mean_servo_tracking_error_deg']:.1f}deg "
            f"force_sat={trained['force_limit_fraction']:.0%} "
            f"candidate={candidate}"
        )
    report = {
        "policy_file": str(a.policy),
        "experiment": "trained PPO versus zero-action control",
        "simulation_only": True,
        "silicone_model_calibrated": False,
        "motor_mapping_verified": False,
        "stage": "diagnosis; never use these weights directly on a real robot",
        "comparison": rows,
    }
    output = ROOT / "output" / "rl_runs" / (
        "zero_vs_ppo_" + a.policy.stem + "_" + a.direction + ".json"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    print("Diagnostic report:", output)
    print("High reward alone is insufficient: compare velocity errors, falls, and action size.")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        sys.exit(1)
