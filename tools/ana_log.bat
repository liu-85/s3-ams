@echo off
setlocal
rem =====================================================================
rem  ana_log.bat -- summarize a mon485 log into one readable page
rem
rem  Usage: ana_log.bat                       (newest log in tools\logs)
rem         ana_log.bat <path-to-log>
rem         ana_log.bat <path-to-log> 15:06:09 15:06:12   (time window)
rem
rem  Shows: overview / printer state timeline / motor events / anomalies
rem =====================================================================

chcp 65001 >nul 2>&1

python "%~dp0ana_log.py" %*
echo.
pause >nul
