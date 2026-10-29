@echo off
setlocal
cd /d "%~dp0"
if not exist "build\robot.exe" (
    echo Build first: run build.bat from this directory.
    pause
    exit /b 1
)
"build\robot.exe"
if errorlevel 1 (
    echo Robot exited with an error. See the message above.
    pause
    exit /b 1
)
