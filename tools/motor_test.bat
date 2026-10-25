@echo off
setlocal
rem =====================================================================
rem  motor_test.bat —— 电机检测（进料/退料点动 + 电流/方向判定）
rem
rem  用法：  motor_test.bat [IP] [通道] [秒] [PWM%]
rem  例：    motor_test.bat                      (默认 192.168.2.100 通道1 1.5s 100%)
rem         motor_test.bat 192.168.2.100 3 2 80
rem
rem  判定逻辑：
rem    转      电机方向变成目标方向 且 峰值电流 >= 150mA
rem    方向给了但没电流  PWM/离合没推动（占空比不够或离合没吸合）
rem    方向都没给        软件没驱动（固件/协议问题）
rem =====================================================================
set "AMS_IP=%~1"
if "%AMS_IP%"=="" set "AMS_IP=192.168.2.100"
set "AMS_CH=%~2"
if "%AMS_CH%"=="" set "AMS_CH=1"
set "AMS_SEC=%~3"
if "%AMS_SEC%"=="" set "AMS_SEC=1.5"
set "AMS_PCT=%~4"
if "%AMS_PCT%"=="" set "AMS_PCT=100"

powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0motor_test.ps1"
exit /b %ERRORLEVEL%
