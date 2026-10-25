@echo off
setlocal
rem =====================================================================
rem  mon485.bat -- 485 monitor (printer frames + our replies + WEB log + motor)
rem
rem  Usage: mon485.bat [IP] [poll_ms]
rem  e.g.   mon485.bat                      (default 192.168.2.100 / 50ms)
rem         mon485.bat 192.168.2.100 50
rem         mon485.bat 192.168.2.100 20    (use this if you see frame loss)
rem
rem  Everything is also written to tools\logs\mon485_yyyyMMdd_HHmmss.log
rem  so nothing is lost when the console scrolls. Open it with Notepad/VSCode.
rem
rem  Colors:
rem    Cyan    = RX  printer -> device
rem    Green   = TX  device  -> printer
rem    Gray    = status line / WEB log
rem    Yellow  = motor action event (dir / clutch / action / print phase changed)
rem    Red     = error log / frame-loss warning
rem
rem  Every frame carries a #seq number. If you see a red "frame loss" line,
rem  the device's 32-frame ring buffer was overrun - lower the 2nd argument.
rem
rem  Requires firmware v1.1.3+ (/bus485raw endpoint).
rem  Ctrl+C to stop.
rem =====================================================================

chcp 65001 >nul 2>&1

set "AMS_IP=%~1"
if "%AMS_IP%"=="" set "AMS_IP=192.168.2.100"
set "AMS_MS=%~2"
if "%AMS_MS%"=="" set "AMS_MS=50"

echo.
echo   AMS IP    : %AMS_IP%
echo   Poll      : %AMS_MS% ms
echo   Log dir   : %~dp0logs
echo.

powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0mon485.ps1" -Ip "%AMS_IP%" -Ms %AMS_MS%
set RC=%ERRORLEVEL%

echo.
echo   Monitor exited (code %RC%). Press any key to close.
pause >nul
exit /b %RC%
