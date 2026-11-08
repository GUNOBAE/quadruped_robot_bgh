@echo off
setlocal EnableExtensions
cd /d "%~dp0"
title BGH Robot Dog - NEW PC INSTALLER
echo ===========================================================
echo BGH QUADRUPED: NEW PC SETUP (Fusion CAD -> MuJoCo)
echo Source model: ASSAMBLE_A / 36 As-Built Joints / 8 loops
echo NOT REAL WALKING; NO RL OR HARDWARE CONTROL
echo ===========================================================
echo.

if not exist ".venv\Scripts\python.exe" (
  where py >nul 2>nul
  if errorlevel 1 (
    echo [ERROR] Windows Python launcher not found.
    echo Install Python 3.12 and select 'Add Python to PATH'.
    goto failed
  )
  echo [1/6] Creating a local Python environment...
  py -3 -m venv .venv
  if errorlevel 1 goto failed
)

set "PY=.venv\Scripts\python.exe"
echo [2/6] Checking MuJoCo and numeric packages...
"%PY%" -c "import mujoco, numpy, scipy" >nul 2>nul
if errorlevel 1 (
  echo Installing required packages. Internet is needed for this step.
  "%PY%" -m pip install -r requirements_new_pc.txt
  if errorlevel 1 goto failed
)

echo [3/6] Restoring the 37-occurrence, 36-joint reference...
if not exist "audit_excerpt_reference.json" (
  "%PY%" make_seed.py
  if errorlevel 1 goto failed
) else (
  echo Reference file already exists; keeping the current version.
)

echo [4/6] Building the fixed-base linkage and 8 loop constraints...
"%PY%" build_preview.py audit_excerpt_reference.json --out output
if errorlevel 1 goto failed

echo [5/6] Generating and verifying the 97-frame kinematic replay...
"%PY%" solve_motion.py
if errorlevel 1 goto failed
"%PY%" view_preview.py --verify-only
if errorlevel 1 goto failed
echo.
echo KINEMATIC BASELINE READY.
echo.

if not exist "Quadruped_CAD_Meshes.zip" (
  echo [6/6] Original Fusion CAD mesh ZIP not found.
  echo.
  echo You must obtain Quadruped_CAD_Meshes.zip on this PC:
  echo 1. Open ASSAMBLE_A.f3d in Fusion 360.
  echo 2. Run fusion_export_meshes.py as a Fusion SCRIPT.
  echo 3. Copy Downloads\Quadruped_CAD_Meshes.zip beside this CMD.
  echo 4. Run NEW_PC_START.cmd again.
  echo.
  echo No real Fusion CAD meshes will be displayed yet.
  echo To see the previous schematic preview instead, run:
  echo   .venv\Scripts\python.exe view_preview.py
  echo.
  goto done
)

echo [6/6] Attaching the ORIGINAL Fusion CAD mesh ZIP...
"%PY%" attach_cad_meshes.py
if errorlevel 1 (
  echo CAD fit failed: output\cad_fit_report.json has details.
  goto failed
)
"%PY%" view_cad_preview.py --verify-only
if errorlevel 1 goto failed
echo.
echo CAD compiled successfully. Opening the MuJoCo CAD viewer...
"%PY%" view_cad_preview.py
if errorlevel 1 goto failed

:done
echo.
echo Setup or preview finished.
pause
exit /b 0

:failed
echo.
echo [ERROR] Setup was stopped. Take a photo of the error text.
echo If available, send output\cad_fit_report.json too.
pause
exit /b 1
