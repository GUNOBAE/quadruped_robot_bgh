"""Train an experimental PPO policy for forward/back/strafe/turn commands.

This is SIMULATION ONLY. Physically verified servo IDs, joint feedback and
silicone material characterization are not available in this training setup.
Do not deploy these weights to Raspberry Pi or physical motors.
"""
from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.env_checker import check_env
import torch

from rl_env import BGHCommandEnv

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "output" / "rl_runs"


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--stage", choices=("stand", "foreaft", "all"), default="stand")
    ap.add_argument("--obs-mode", choices=("sim", "imu"), default="sim")
    ap.add_argument("--steps", type=int, default=10000)
    ap.add_argument("--seed", type=int, default=21)
    ap.add_argument("--load", type=Path, default=None,
                    help="Continue PPO from an earlier training stage")
    ap.add_argument("--save", type=Path, default=None,
                    help="Output policy ZIP. Defaults to output/rl_runs/<stage>_<obs-mode>.zip")
    ap.add_argument("--smoke", action="store_true",
                    help="Lightweight PPO smoke test with short rollout")
    args = ap.parse_args()
    if not 64 <= args.steps <= 10_000_000:
        ap.error("--steps must be 64 .. 10,000,000")
    xml = ROOT / "output" / "physics_rl_v0.xml"
    if not xml.is_file():
        raise FileNotFoundError(
            "Run RUN_RL_CHECK.cmd first: output/physics_rl_v0.xml missing"
        )
    OUT.mkdir(parents=True, exist_ok=True)
    env_raw = BGHCommandEnv(
        xml_path=xml, stage=args.stage, obs_mode=args.obs_mode,
        max_seconds=8., randomize_contact=not args.smoke
    )
    # The real robot's PWM servos do not report joint angle/velocity:
    # full-state sim policies must not be transferred to hardware unchanged.
    print("OBSERVATION MODE =", args.obs_mode)
    print("SIM ONLY. ACTIONS = 12 PROVISIONAL SERVOS, NOT VERIFIED CHANNELS.")
    check_env(env_raw, warn=True, skip_render_check=True)
    env = Monitor(env_raw)
    torch.set_num_threads(min(2, torch.get_num_threads()))
    rollout = 64 if args.smoke else 512
    batch = 32 if args.smoke else 128
    if args.load:
        if not args.load.is_file():
            raise FileNotFoundError(str(args.load))
        model = PPO.load(str(args.load), env=env, device="cpu")
        if model.observation_space != env.observation_space:
            raise RuntimeError("Checkpoint observation mode does not match")
        reset_steps = False
    else:
        model = PPO(
            "MlpPolicy", env, verbose=1, seed=args.seed, device="cpu",
            n_steps=rollout, batch_size=batch, n_epochs=(2 if args.smoke else 5),
            gamma=.99, gae_lambda=.95, learning_rate=3e-4,
            clip_range=.2, ent_coef=.005,
            policy_kwargs={"net_arch": [64, 64]},
        )
        reset_steps = True
    save = (args.save or
            OUT / f"{args.stage}_{args.obs_mode}_ppo.zip").resolve()
    save.parent.mkdir(parents=True, exist_ok=True)
    callback = CheckpointCallback(
        save_freq=5000 if args.smoke else 25000,
        save_path=str(save.parent),
        name_prefix=f"{args.stage}_{args.obs_mode}_checkpoint",
        save_replay_buffer=False, save_vecnormalize=False,
    )
    interrupted = False
    try:
        model.learn(total_timesteps=args.steps,
                    reset_num_timesteps=reset_steps,
                    progress_bar=False, callback=callback)
    except KeyboardInterrupt:
        interrupted = True
        print("Training interrupted. Saving current checkpoint safely.")
    finally:
        model.save(str(save))
        env.close()
    result = {
        "stage": args.stage,
        "obs_mode": args.obs_mode,
        "steps_requested": args.steps,
        "model_num_timesteps": int(model.num_timesteps),
        "interrupted": interrupted,
        "policy_file": str(save),
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "simulation_only": True,
        "physical_walking_validated": False,
        "provisional_servo_actuators": True,
        "silicone_contact_is_a_proxy": True,
        "note": "Training completion does NOT demonstrate walking. Test each commanded direction.",
    }
    metadata = save.with_suffix(".json")
    metadata.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print("POLICY SAVED:", save)
    print("TRAINING RECORD:", metadata)
    if interrupted:
        print("Resume later with --load", save)
    else:
        print("NEXT: run evaluate_rl.py and verify direction-specific motion/fall rates.")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        sys.exit(1)
