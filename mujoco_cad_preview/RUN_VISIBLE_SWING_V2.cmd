@echo off
setlocal EnableExtensions
cd /d "%~dp0"
title BGH Quadruped - Visible IK Swing V2
echo ================================================================
echo VISIBLE SINGLE LEG SWING V2
echo Default: LIFT 22mm, MOVE FORWARD 20mm, REPEAT UNTIL YOU CLOSE WINDOW
echo Body FIXED. Gravity/contact OFF. NOT a walking simulation.
echo Original Fusion meshes / PPO weights are unchanged.
echo ================================================================
if not exist ".venv\Scripts\python.exe" (
 echo ERROR: Run earlier MuJoCo setup first.
 goto failed
)
if not exist "output\physics_v1.xml" (
 echo ERROR: Missing Physics V1 model; first run original physics setup.
 goto failed
)
if not exist "swing_ik_v1.py" (
 echo ERROR: Missing predecessor swing_ik_v1.py.
 goto failed
)
if not exist "swing_ik_v2.py" (
 echo ERROR: Missing swing_ik_v2.py. Download it from GitHub.
 goto failed
)
echo 1 - FL: front-left leg
echo 2 - FR: front-right leg
echo 3 - BL: back-left leg
echo 4 - BR: back-right leg
choice /C 1234 /N /M "Select one leg (1-4): "
if errorlevel 4 (set "LEG=BR" & goto selected)
if errorlevel 3 (set "LEG=BL" & goto selected)
if errorlevel 2 (set "LEG=FR" & goto selected)
set "LEG=FL"
:selected
echo.
echo Selected leg: %LEG%
echo Check the GREEN moving target against the actual toe.
echo A 22mm swing is over THREE TIMES the previous 6mm target.
echo Keep the MuJoCo window OPEN to see repeated cycles.
echo Close MuJoCo window to save the report for the FIRST cycle.
echo.
".venv\Scripts\python.exe" swing_ik_v2.py --leg %LEG% --view
if errorlevel 1 goto failed
echo.
echo Read:
echo output\ik_swing_visible_v2_report.json
echo output\ik_swing_visible_v2_trajectory.csv
pause
exit /b 0
:failed
echo.
echo STOPPED. Copy the exact Python error text.
echo Do NOT change actuator ranges on a real servo.
pause
exit /b 1
