@echo off
title E7-RCU Downloader

REM 接收主脚本传过来的参数（去除两边的双引号）
set "BIN_FULL_PATH=%~1"
set "TARGET_NAME=%~2"

REM 增加调试信息，方便看清到底接到了什么
echo ==========================================
echo           E7-RCU Download Tool            
echo ==========================================
echo [DEBUG] Received Arg1: "%BIN_FULL_PATH%"
echo [DEBUG] Received Arg2: "%TARGET_NAME%"
echo ==========================================

if "%BIN_FULL_PATH%"=="" (
    echo [ERROR] No input file specified for download.
    
    exit
)

REM 初始化30秒超时计数器
set /a TIMEOUT_COUNT=0

:WAIT_DRIVE
set "E7_DRIVE="
for %%d in (D E F G H I J K L M N O P Q R S T U V W X Y Z) do (
    if exist "%%d:\RCUcdc.inf" set "E7_DRIVE=%%d:"
)

if "%E7_DRIVE%"=="" (
    REM 检查是否超过30秒
    if %TIMEOUT_COUNT% geq 1 (
        echo.
        echo [ERROR] Timeout! E7-RCU disk not found within 30 seconds.
        echo [STATUS] Auto exiting...
        
        exit
    )

    echo [%time%] Waiting for E7-RCU disk... (%TIMEOUT_COUNT%/30s)
    timeout /t 1 > nul
    
    set /a TIMEOUT_COUNT+=1
    goto WAIT_DRIVE
)

echo.
echo [INFO] E7-RCU found on %E7_DRIVE%
echo [INFO] Copying to %E7_DRIVE%\%TARGET_NAME%...

copy /Y "%BIN_FULL_PATH%" "%E7_DRIVE%\%TARGET_NAME%"

if %errorlevel% equ 0 (
    echo [SUCCESS] File copied successfully!
    
    exit
) else (
    echo [ERROR] Copy failed.
    
    exit
)