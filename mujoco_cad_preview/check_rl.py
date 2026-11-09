"""Quick local safety/compatibility checks before spending CPU on PPO.

Prints the preliminary zero-command standing behavior and checks
Gymnasium step/reset, six commands, 4 feet, and 12 actuators.
Passing this does NOT establish actual walking or real-world validity.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import numpy as np
from stable_baselines3.common.env_checker import check_env
from rl_env import BGHCommandEnv, DIRECTIONS, DEFAULT_XML

ROOT = Path(__file__).resolve().parent


def main():
    if not DEFAULT_XML.is_file():
        raise FileNotFoundError("Missing physics_rl_v0.xml. Run build_rl_model.py first.")
    env = BGHCommandEnv(stage="stand", obs_mode="sim", randomize_contact=False,
                        max_seconds=3.0)
    try:
        check_env(env, warn=True, skip_render_check=True)
        initial, _ = env.reset(seed=2026)
        first_height = float(env.data.qpos[2])
        stopped_at = None
        contacts = 0
        total = 0
        for i in range(env.max_steps):
            obs, reward, terminated, truncated, info = env.step(
                np.zeros(12, dtype=np.float32)
            )
            assert np.all(np.isfinite(obs)) and math.isfinite(reward)
            contacts = max(contacts, info["feet_in_contact"])
            total += 1
            if terminated or truncated:
                stopped_at = {"elapsed_s": total * env.dt,
                              "fall": bool(info["fall"]),
                              "height": info["body_height_m"],
                              "tilt": info["tilt_deg"]}
                break
        direction_checks = {}
        for direction in DIRECTIONS:
            env.fixed_command = direction
            env.reset(seed=2026)
            _, _, term, _, info = env.step(np.zeros(12, dtype=np.float32))
            direction_checks[direction] = {
                "command": env.command.tolist(),
                "first_step_fall": bool(term),
                "height_m": info["body_height_m"],
            }
        report = {
            "model": str(DEFAULT_XML),
            "gym_check_env_passed": True,
            "action_count": 12,
            "base_height_initial_m": first_height,
            "zero_action_simulated_seconds": total * env.dt,
            "zero_action_max_contacts": contacts,
            "zero_action_final": stopped_at,
            "direction_commands": direction_checks,
            "status": ("ZERO_ACTION_BASELINE_STABLE_CANDIDATE"
                       if stopped_at and not stopped_at["fall"]
                       else "ZERO_ACTION_FALLS_OR_EARLY_STOP"),
            "warning": ("Passing this API test only confirms executable simulation; "
                        "no walking is learned or validated here."),
        }
        path = ROOT / "output" / "rl_v0_preflight.json"
        path.write_text(json.dumps(report, indent=2, ensure_ascii=False),
                        encoding="utf-8")
        print(json.dumps({
            "gym_check_env_passed": True,
            "zero_action_final": stopped_at,
            "max_foot_contacts": contacts,
            "status": report["status"],
        }, indent=2))
        print("Preflight report:", path)
        if stopped_at and stopped_at["fall"]:
            print("WARNING: Zero-command pose falls. Fix Physics V1 before expecting reliable walking.")
    finally:
        env.close()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        sys.exit(1)
