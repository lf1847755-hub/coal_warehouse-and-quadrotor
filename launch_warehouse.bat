@echo off
setlocal
cd /d "%~dp0"
set ISAACSIM_PATH=E:\isaacsim
set PEGASUS_DISABLE_ROS2=1
call "%ISAACSIM_PATH%\python.bat" -u "%~dp0warehouse_app.py" --/log/file="%~dp0logs\runtime_kit.log" > "%~dp0logs\runtime_console.log" 2>&1
endlocal
