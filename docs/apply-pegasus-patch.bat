@echo off
setlocal
REM ============================================================
REM  Apply the two local Pegasus 5.1.0 edits for the
REM  "Coal Warehouse - Manual Flight" Isaac Sim project.
REM
REM  Usage:
REM    apply-pegasus-patch.bat "D:\path\to\PegasusSimulator\extensions\pegasus.simulator"
REM  If no argument is given, the path below is used.
REM  Requires Git for Windows (git on PATH) to apply the patch.
REM ============================================================

set "PEGASUS_EXT=%~1"
if "%PEGASUS_EXT%"=="" set "PEGASUS_EXT=E:\isaacsim\PegasusSimulator\extensions\pegasus.simulator"
set "PATCH=%~dp0pegasus-5.1.0-local-changes.patch"

if not exist "%PATCH%" (
    echo [ERROR] Patch file not found next to this script: "%PATCH%"
    exit /b 1
)

if not exist "%PEGASUS_EXT%\pegasus\simulator\params.py" (
    echo [ERROR] Pegasus extension not found at: "%PEGASUS_EXT%"
    echo         Pass the folder ...\PegasusSimulator\extensions\pegasus.simulator as the first argument.
    exit /b 1
)

where git >nul 2>nul
if errorlevel 1 (
    echo [ERROR] git was not found on PATH. Install Git for Windows, or edit the two files by hand
    echo         ^(see section 4 of IsaacSim-Pegasus-项目移交.md^).
    exit /b 1
)

echo Applying patch to: "%PEGASUS_EXT%"

git -c core.autocrlf=false -C "%PEGASUS_EXT%" apply --check -p1 --whitespace=nowarn "%PATCH%" >nul 2>nul
if errorlevel 1 (
    findstr /c:"skip_check=True" "%PEGASUS_EXT%\pegasus\simulator\params.py" >nul 2>nul
    if not errorlevel 1 (
        echo [SKIP] The patch looks already applied - nothing to do.
        exit /b 0
    )
    echo [WARN] Strict check failed; retrying with --ignore-whitespace ...
    git -c core.autocrlf=false -C "%PEGASUS_EXT%" apply -p1 --whitespace=nowarn --ignore-whitespace "%PATCH%"
    if errorlevel 1 goto :failed
) else (
    git -c core.autocrlf=false -C "%PEGASUS_EXT%" apply -p1 --whitespace=nowarn "%PATCH%"
    if errorlevel 1 goto :failed
)

echo.
echo [OK] Patch applied. Verify with:
echo      findstr /c:"skip_check=True" "%PEGASUS_EXT%\pegasus\simulator\params.py"
echo      findstr /c:"PEGASUS_DISABLE_ROS2" "%PEGASUS_EXT%\pegasus\simulator\logic\backends\__init__.py"
exit /b 0

:failed
echo.
echo [ERROR] Patch did not apply. The Pegasus source may be a different version.
echo         Compare with the manual snippets in IsaacSim-Pegasus-项目移交.md (section 4).
exit /b 1