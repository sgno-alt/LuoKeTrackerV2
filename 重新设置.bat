@echo off
cd /d "%~dp0"
call "%~dp0_env.bat"
if errorlevel 1 goto :end
python luoke_tracker.py --setup
:end
pause
