@echo off
setlocal
set "APPDIR=%~dp0"
set "EXE=%APPDIR%target\release\ScanImprover.exe"

if not exist "%EXE%" (
    echo ScanImprover.exe not found - building release...
    cd /d "%APPDIR%"
    cargo build --release
)

if exist "%EXE%" (
    start "" "%EXE%" %*
) else (
    echo.
    echo Build failed. Is Rust installed? Run: rustup default stable
    pause
)

endlocal
