@echo off
rem ===========================================================
rem  Inverted Pendulum - Modelling and Simulation
rem  (ASCII only on purpose: cmd.exe parses .bat with the OEM
rem   codepage, so non-ASCII text here corrupts the script.)
rem  Results: console output + out\cascade_run.png
rem ===========================================================
chcp 65001 >nul
cd /d "%~dp0"
set PYTHONIOENCODING=utf-8
set MPLCONFIGDIR=%~dp0out\.mplcache

echo.
echo === Inverted Pendulum Simulation ===
echo script dir : %~dp0
echo takes 1-3 minutes, do not close this window
echo.

python "%~dp0pendulum_design.py"

if errorlevel 1 (
    echo.
    echo [FAILED] send me the error text above.
    echo   If it says "python is not recognized", Python is not in PATH.
    echo   Workaround: drag pendulum_design.py onto python.exe
)

echo.
echo === finished ===
echo plots : %~dp0out\cascade_run.png
echo         %~dp0out\with_sensors.png
echo gains : %~dp0out\gains.json
echo.
pause
