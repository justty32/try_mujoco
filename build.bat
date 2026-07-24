@echo off
setlocal

:: Add MinGW to PATH
set PATH=C:\dev\mingw64\bin;%PATH%

cmake -G "MinGW Makefiles" ^
    -DCMAKE_C_COMPILER=C:\dev\mingw64\bin\gcc.exe ^
    -B build -S . ^
    && cmake --build build --config Release

if %ERRORLEVEL% EQU 0 (
    echo.
    echo Build succeeded!
    echo Run from project root: build\robot.exe
) else (
    echo.
    echo Build failed.
)
