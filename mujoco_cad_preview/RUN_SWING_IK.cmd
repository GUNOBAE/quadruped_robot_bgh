@echo off
setlocal EnableExtensions
cd /d "%~dp0"
title BGH Robot Dog - Single-Leg IK Swing (Simulation Only)
echo =====================================================
echo ONE LEG SWING: closed-loop numerical IK bench
echo Torso FIXED, gravity OFF, foot contact OFF
echo No retraining, no real servo commands, no hardware I/O
echo =====================================================
if not exist ".venv\Scripts\python.exe" (
  echo ERROR: Python environment missing. Previous MuJoCo setup required.
  goto failed
)
if not exist "output\physics_v1.xml" (
  echo ERROR: Physics model missing. Run RUN_PHYSICS_V1.cmd first.
  goto failed
)
if not exist "probe_actuators.py" (
  echo ERROR: Download probe_actuators.py from the current GitHub branch.
  goto failed
)
if not exist "swing_ik_v1.py" (
  echo ERROR: Download swing_ik_v1.py from the current GitHub branch.
  goto failed
)
echo 1 - FL front-left (recommended first test)
echo 2 - FR front-right
echo 3 - BL back-left
echo 4 - BR back-right
choice /C 1234 /N /M "Choose one leg (1-4): "
if errorlevel 4 (set "LEG=BR" & goto selected)
if errorlevel 3 (set "LEG=BL" & goto selected)
if errorlevel 2 (set "LEG=FR" & goto selected)
set "LEG=FL"
:selected
echo.
echo Selected leg: %LEG%
echo Target: lift 6mm, forward 5mm, return to floor height in 2 seconds.
echo Target marker in viewer: green sphere.
echo.
".venv\Scripts\python.exe" swing_ik_v1.py --leg %LEG% --view
if errorlevel 1 goto failed
echo.
echo View output\ik_swing_report.json and output\ik_swing_trajectory.csv
pause
exit /b 0
:failed
echo.
echo TEST STOPPED - this is not a physical walking test.
echo Copy the console error; preserve the original CAD and PPO files.
pause
exit /b 1
