@echo off
setlocal EnableExtensions
cd /d "%~dp0"
title BGH Robot Dog - Walking Failure Diagnosis
echo ============================================================
echo BGH QUADRUPED: WHY DOES IT CROUCH INSTEAD OF WALKING?
echo TEST A: Fixed-base +/- 0.12 rad actuator-to-foot mapping
echo TEST B: PPO policy versus zero-action on identical physics
echo No retraining, no changes to source CAD or real robot hardware.
echo ============================================================
if not exist ".venv\Scripts\python.exe" (
 echo [ERROR] Missing Python virtual environment. Use prior setup first.
 goto failed
)
if not exist "output\physics_rl_v0.xml" (
 echo [ERROR] Missing RL physics XML. Run RUN_RL_CHECK.cmd first.
 goto failed
)
if not exist "output\rl_runs\all_sim_ppo.zip" (
 echo [ERROR] Trained PPO checkpoint missing.
 goto failed
)
if not exist "probe_actuators.py" (
 echo [ERROR] Download probe_actuators.py from the diagnostic branch.
 goto failed
)
set "PY=.venv\Scripts\python.exe"
echo.
echo [1/2] Testing the actuator-to-foot response with body fixed...
"%PY%" probe_actuators.py
if errorlevel 1 goto failed
echo.
echo [2/2] Comparing trained AI against zero-command baseline...
"%PY%" compare_to_zero.py --policy output\rl_runs\all_sim_ppo.zip --seconds 6
if errorlevel 1 goto failed
echo.
echo DONE. Read:
echo output\rl_runs\actuator_probe.json
echo output\rl_runs\zero_vs_ppo_all_sim_ppo_all.json
echo.
echo No new policy was trained. This is only a simulation diagnostic.
pause
exit /b 0
:failed
echo.
echo TEST STOPPED. Preserve your original weights and CAD files.
echo Send the exact error shown above; do not increase torque/reward yet.
pause
exit /b 1
