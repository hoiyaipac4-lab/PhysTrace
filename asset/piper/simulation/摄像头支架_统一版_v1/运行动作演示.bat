@echo off
setlocal
if not defined ISAAC_SIM_PATH (
 echo Set ISAAC_SIM_PATH to your initialized Isaac Sim installation.
 exit /b 1
)
if not exist "%ISAAC_SIM_PATH%\python.bat" (
 echo Set ISAAC_SIM_PATH to your Isaac Sim installation folder.
 pause
 exit /b 1
)
call "%ISAAC_SIM_PATH%\python.bat" "%~dp0run_preview.py" --motion
if errorlevel 1 pause
