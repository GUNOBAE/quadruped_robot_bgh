@echo off
setlocal EnableExtensions
cd /d "%~dp0"
title BGH Quadruped - PPO RL Preflight (SIM ONLY)
echo =========================================================
echo BGH Robot Dog - Directional PPO RL Preflight
echo Goal: forward/back/sideways/turn. NO REAL ROBOT CONTROL.
echo =========================================================
if not exist ".venv\Scripts\python.exe" (
  echo ERROR: Run NEW_PC_START.cmd to create MuJoCo Python environment.
  goto failed
)
if not exist "output\physics_v1.xml" (
  echo ERROR: Physics V1 missing. Run RUN_PHYSICS_V1.cmd once.
  goto failed
)
if not exist "output\physics_v1_config.json" (
  echo ERROR: Physics V1 config missing.
  goto failed
)
set "PY=.venv\Scripts\python.exe"

echo [1/4] Installing PPO / Gymnasium / PyTorch, if needed...
"%PY%" -c "import stable_baselines3, gymnasium, torch" >nul 2>nul
if errorlevel 1 (
  "%PY%" -m pip install -r requirements_rl.txt
  if errorlevel 1 (
    echo Python and Torch wheel mismatch. Use Python 3.12-3.14 in
    echo a dedicated environment if current Python is unsupported.
    goto failed
  )
)
echo [2/4] Building silicone-soft-contact physics approximation...
"%PY%" build_rl_model.py
if errorlevel 1 goto failed
echo [3/4] Testing Gymnasium env, 6 movement commands and contact physics...
"%PY%" check_rl.py
if errorlevel 1 goto failed
echo [4/4] Running a very small PPO training smoke test...
"%PY%" train_rl.py --stage stand --steps 64 --smoke --save output\rl_runs\smoke_sim_ppo.zip
if errorlevel 1 goto failed
echo.
echo RL PIPELINE CHECK PASSED. No trained walking yet.
echo Data: output\rl_v0_preflight.json
echo Run RUN_RL_TRAIN.cmd to begin staged learning.
pause
exit /b 0
:failed
echo.
echo RL PRECHECK FAILED. Copy console log and output\rl_v0_preflight.json if present.
pause
exit /b 1
