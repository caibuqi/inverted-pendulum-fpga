@echo off
rem ===========================================================
rem  Inverted Pendulum - Diagnostics and Parameter Sweeps
rem  (ASCII only on purpose - see run_sim.bat)
rem ===========================================================
chcp 65001 >nul
cd /d "%~dp0"
set PYTHONIOENCODING=utf-8
set MPLCONFIGDIR=%~dp0out\.mplcache

echo.
echo === Inverted Pendulum Diagnostics ===
echo takes 2-5 minutes, do not close this window
echo.

python "%~dp0diag2.py"

if errorlevel 1 (
    echo.
    echo [FAILED] send me the error text above.
)

echo.
echo === finished ===
echo.
pause
