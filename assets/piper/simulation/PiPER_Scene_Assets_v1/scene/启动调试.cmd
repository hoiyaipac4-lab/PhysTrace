@echo off
setlocal
if not defined ISAAC_SIM_DIR (
 echo Set ISAAC_SIM_DIR to your initialized Isaac Sim installation.
 exit /b 1
)
if not exist "%ISAAC_SIM_DIR%\python.bat" (
 echo Set ISAAC_SIM_DIR to your Isaac Sim installation.
 pause
 exit /b 1
)
call "%ISAAC_SIM_DIR%\python.bat" "%~dp0run_debug.py"
endlocal
