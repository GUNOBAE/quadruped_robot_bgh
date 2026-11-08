@echo off
setlocal EnableExtensions
cd /d "%~dp0"
echo =========================================================
echo BGH Robot Dog - Real Fusion CAD Mesh Preview
echo VISUAL / KINEMATIC ONLY -- NOT WALKING OR RL
echo =========================================================

if not exist "output\quadruped_preview.xml" (
  echo ERROR: Missing output\quadruped_preview.xml
  echo Copy these files into your EXISTING quadruped_v01 folder.
  goto fail
)
if not exist "output\model_manifest.json" (
  echo ERROR: Missing output\model_manifest.json
  goto fail
)
if not exist "audit_excerpt_reference.json" (
  echo ERROR: Missing audit_excerpt_reference.json
  goto fail
)
if not exist "Quadruped_CAD_Meshes.zip" (
  echo ERROR: Missing Quadruped_CAD_Meshes.zip
  echo Copy the original ZIP from Fusion 360 into this folder.
  goto fail
)
if not exist ".venv\Scripts\python.exe" (
  where py >nul 2>nul
  if errorlevel 1 (
    echo ERROR: Python launcher 'py' not found.
    goto fail
  )
  py -3 -m venv .venv
  if errorlevel 1 goto fail
)
set "PY=.venv\Scripts\python.exe"
"%PY%" -c "import mujoco, numpy" >nul 2>nul
if errorlevel 1 (
  echo Installing MuJoCo and numpy. Internet access needed.
  "%PY%" -m pip install "mujoco>=3.3.7,<4" "numpy>=1.26,<3"
  if errorlevel 1 goto fail
)

echo [1/2] Checking all 37 STL parts and fitting CAD coordinates...
"%PY%" attach_cad_meshes.py
if errorlevel 1 (
  echo Read output\cad_fit_report.json for part-by-part errors.
  goto fail
)

echo [2/2] Checking the MuJoCo model and launching the viewer...
"%PY%" view_cad_preview.py
if errorlevel 1 goto fail
exit /b 0

:fail
echo.
echo Preview stopped. Send the error and output\cad_fit_report.json.
pause
exit /b 1
