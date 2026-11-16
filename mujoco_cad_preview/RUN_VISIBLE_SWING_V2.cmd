@echo off
setlocal EnableExtensions
cd /d "%~dp0"
title BGH Quadruped - Visible IK Swing V2
echo ================================================================
echo VISIBLE SINGLE LEG SWING V2
echo Body FIXED. Gravity and contact OFF. Not walking.
echo ================================================================
if not exist ".venv\Scripts\python.exe" goto missing_python
if not exist "output\physics_v1.xml" goto missing_model
if not exist "probe_actuators.py" goto missing_helper
if not exist "swing_ik_v2.py" goto missing_script
choice /C 1234 /N /M "Leg: 1 FL, 2 FR, 3 BL, 4 BR: "
if errorlevel 4 goto leg_br
if errorlevel 3 goto leg_bl
if errorlevel 2 goto leg_fr
set "LEG=FL"
goto run
:leg_br
set "LEG=BR"
goto run
:leg_bl
set "LEG=BL"
goto run
:leg_fr
set "LEG=FR"
:run
".venv\Scripts\python.exe" swing_ik_v2.py --leg %LEG% --view
if errorlevel 1 goto failed
pause
exit /b 0
:missing_python
echo ERROR: Python environment missing.
goto failed
:missing_model
echo ERROR: output\physics_v1.xml missing.
goto failed
:missing_helper
echo ERROR: probe_actuators.py missing.
goto failed
:missing_script
echo ERROR: swing_ik_v2.py missing.
:failed
echo STOPPED. Preserve the console error text.
pause
exit /b 1
