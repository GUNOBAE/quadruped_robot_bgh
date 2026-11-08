@echo off
setlocal EnableExtensions
cd /d "%~dp0"
title BGH QUADRUPED - PHYSICS V1 (UNVALIDATED)
echo ==========================================================
echo BGH Robot Dog - PHYSICS V1
echo GRAVITY + FLOOR CONTACT + 4 TOE COLLIDERS + SERVO PD
echo THIS IS NOT RL OR VERIFIED STANDING / NO ROBOT HARDWARE
echo ==========================================================
echo.

if not exist ".venv\Scripts\python.exe" (
  echo ERROR: No Python virtual environment.
  echo Run NEW_PC_START.cmd first to install MuJoCo.
  goto failed
)
if not exist "output\cad_preview.xml" (
  echo ERROR: No original Fusion CAD MuJoCo preview.
  echo First get 37 CAD parts working using NEW_PC_START.cmd.
  goto failed
)
if not exist "output\cad_fit_report.json" (
  echo ERROR: CAD alignment report missing.
  goto failed
)

set "PY=.venv\Scripts\python.exe"

echo [1/3] Building provisional physics model from your real CAD...
"%PY%" build_physics_v1.py
if errorlevel 1 goto failed

echo [2/3] Running 3s gravity/contact diagnostics (no viewer)...
"%PY%" run_physics_v1.py --seconds 3
if errorlevel 1 (
  echo Physics stepping failed. Record the console error text.
  goto failed
)

echo.
echo The diagnostics report is output\physics_v1_report.json
echo NOTE: If NOT_STANDING_YET, that is an engineering result.
echo Do NOT increase servo torque until its real value is measured.
echo.
echo [3/3] Opening Physics V1 live viewer...
"%PY%" run_physics_v1.py --seconds 3 --view
if errorlevel 1 goto failed
echo.
echo Viewer closed. Check output\physics_v1_report.json.
pause
exit /b 0

:failed
echo.
echo PHYSICS V1 STOPPED. This does not affect CAD or earlier previews.
echo Send full console output and output\physics_v1_report.json if created.
pause
exit /b 1
