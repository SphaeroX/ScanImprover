@echo off
setlocal
set "APPDIR=%~dp0"

rem Rustup default install location (in case PATH is not set in this shell)
set "PATH=%PATH%;%USERPROFILE%\.cargo\bin"
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
