@echo off
setlocal
set "APPDIR=%~dp0"
cd /d "%APPDIR%"

echo Building ScanImprover (Release)...
cargo build --release
if errorlevel 1 (
    echo.
    echo Build failed!
    pause
    exit /b 1
)

set "EXE=%APPDIR%target\release\ScanImprover.exe"
if exist "%EXE%" (
    echo Starting ScanImprover...
    start "" "%EXE%" %*
) else (
    echo.
    echo Executable not found at %EXE%
    pause
)

endlocal
