@echo off
setlocal
rem =====================================================================
rem  mon485.bat —— 485 通讯监控（打印机原始指令 + 本机回复 + WEB日志）
rem
rem  用法：  mon485.bat [IP] [轮询ms]
rem  例：    mon485.bat                      (默认 192.168.2.100 300ms)
rem         mon485.bat 192.168.2.100 200
rem
rem  字体颜色：
rem    青色 Cyan   = RX  打印机 -> 本机（打印机发来的原始指令）
rem    绿色 Green  = TX  本机 -> 打印机（本机给的回复）
rem    灰色 Gray   = 状态行 / WEB 日志（同样的消息没变化时不刷新）
rem
rem  依赖固件 v1.1.3+（/bus485raw 端点；旧版该功能 404，
rem  脚本会黄字提示并只监控 WEB 日志/状态）
rem  Ctrl+C 停止
rem =====================================================================
set "AMS_IP=%~1"
if "%AMS_IP%"=="" set "AMS_IP=192.168.2.100"
set "AMS_MS=%~2"
if "%AMS_MS%"=="" set "AMS_MS=300"

powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0mon485.ps1"
exit /b %ERRORLEVEL%
