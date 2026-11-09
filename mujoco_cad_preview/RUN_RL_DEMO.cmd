@echo off
setlocal EnableExtensions
cd /d "%~dp0"
title BGH Quadruped PPO - Direction Demo
echo =========================================================
echo BGH Robot Dog: six simulated directional movement commands
echo CAUTION: training output is NOT a robot-safe hardware controller
echo =========================================================
if not exist ".venv\Scripts\python.exe" (
  echo Python environment missing.
  goto failed
)
if not exist "output\rl_runs\all_sim_ppo.zip" (
  echo No trained all-direction checkpoint found.
  echo Run RUN_RL_TRAIN.cmd first.
  goto failed
)
set "PY=.venv\Scripts\python.exe"
echo 0 - Numeric evaluation of ALL 6 directions (no GUI)
echo 1 - Forward
echo 2 - Backward
echo 3 - Sidestep LEFT
echo 4 - Sidestep RIGHT
echo 5 - Rotate LEFT
echo 6 - Rotate RIGHT
choice /C 0123456 /N /M "Choose direction (0..6): "
set "MODE=all"
if errorlevel 7 set "MODE=turn_right" & goto selected
if errorlevel 6 set "MODE=turn_left" & goto selected
if errorlevel 5 set "MODE=right" & goto selected
if errorlevel 4 set "MODE=left" & goto selected
if errorlevel 3 set "MODE=backward" & goto selected
if errorlevel 2 set "MODE=forward" & goto selected
:selected
if "%MODE%"=="all" (
  "%PY%" evaluate_rl.py --direction all --seconds 6
) else (
  "%PY%" evaluate_rl.py --direction "%MODE%" --seconds 8 --view
)
if errorlevel 1 goto failed
echo Direction report written to output\rl_runs\direction_evaluation.json
pause
exit /b 0
:failed
echo.
echo Demo failed or no trained checkpoint. Read the message above.
pause
exit /b 1
