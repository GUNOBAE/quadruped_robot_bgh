@echo off
setlocal EnableExtensions
cd /d "%~dp0"
title BGH Quadruped - PPO Directional Training (SIM ONLY)
echo =========================================================
echo BGH Robot Dog - STAGED PPO DIRECTIONAL TRAINING
echo Stage A: hold standing balance / zero velocity
echo Stage B: forward + backward velocity
echo Stage C: forward/back/left/right/left-turn/right-turn
echo NOT FOR PHYSICAL MOTORS. No real servo mapping or material calibration.
echo =========================================================
if not exist ".venv\Scripts\python.exe" (
  echo ERROR: Run RUN_RL_CHECK.cmd first.
  goto failed
)
if not exist "output\rl_v0_preflight.json" (
  echo ERROR: Run RUN_RL_CHECK.cmd before training.
  goto failed
)
if not exist "output\physics_rl_v0.xml" (
  echo ERROR: Missing silicone-foot RL model.
  goto failed
)
set "PY=.venv\Scripts\python.exe"
if not exist "output\rl_runs" mkdir "output\rl_runs"

echo.
echo [Stage A / 3] Standing (20,000 environment decisions)...
if exist "output\rl_runs\stand_sim_ppo.zip" (
  echo Existing standing policy found; skipping Stage A.
) else (
  "%PY%" train_rl.py --stage stand --steps 20000 --save output\rl_runs\stand_sim_ppo.zip
  if errorlevel 1 goto failed
)
echo.
echo [Stage B / 3] Forward/backward (60000 decisions)...
if exist "output\rl_runs\foreaft_sim_ppo.zip" (
  echo Existing fore-aft policy found; skipping Stage B.
) else (
  "%PY%" train_rl.py --stage foreaft --steps 60000 --load output\rl_runs\stand_sim_ppo.zip --save output\rl_runs\foreaft_sim_ppo.zip
  if errorlevel 1 goto failed
)
echo.
echo [Stage C / 3] All directions (180000 decisions)...
if exist "output\rl_runs\all_sim_ppo.zip" (
  echo Existing all-direction policy found; skipping Stage C.
) else (
  "%PY%" train_rl.py --stage all --steps 180000 --load output\rl_runs\foreaft_sim_ppo.zip --save output\rl_runs\all_sim_ppo.zip
  if errorlevel 1 goto failed
)

echo.
echo =========================================================
echo WEIGHTS SAVED - THIS DOES NOT PROVE THE ROBOT WALKS.
echo Checking direction-by-direction test metrics next...
echo =========================================================
"%PY%" evaluate_rl.py --policy output\rl_runs\all_sim_ppo.zip --direction all --seconds 6
if errorlevel 1 goto failed
echo.
echo See output\rl_runs\direction_evaluation.json for real measurements.
echo Open RUN_RL_DEMO.cmd to choose a direction to watch in MuJoCo.
pause
exit /b 0
:failed
echo.
echo RL TRAINING STOPPED. Copy error; keep all saved checkpoints.
pause
exit /b 1
