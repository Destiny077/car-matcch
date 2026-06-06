@echo off
setlocal EnableExtensions

set "SCRIPT_DIR=%~dp0"
pushd "%SCRIPT_DIR%"

set "TOOLCHAIN=%SCRIPT_DIR%compiler\ARM_v7\bin"
set "PATH=%TOOLCHAIN%;%PATH%"
set "path_lib=%SCRIPT_DIR%compiler\includeRTOSE7"
set "linker_script=%path_lib%\o\STM32H743ZITX_FLASH.ld"

if "%~1"=="" (
    set "path_src=%SCRIPT_DIR%2026v1.c"
    set "path_out_name=%SCRIPT_DIR%2026v1"
) else (
    set "path_src=%~f1"
    set "path_out_name=%~dpn1"
)

echo [INFO] Project: %SCRIPT_DIR%
echo [INFO] Source: %path_src%

if not exist "%TOOLCHAIN%\arm-none-eabi-gcc.exe" (
    echo [ERROR] Missing compiler: %TOOLCHAIN%\arm-none-eabi-gcc.exe
    goto ON_ERROR
)

if not exist "%path_lib%" (
    echo [ERROR] Missing library path: %path_lib%
    goto ON_ERROR
)

if not exist "%linker_script%" (
    echo [ERROR] Missing linker script: %linker_script%
    goto ON_ERROR
)

if exist "%path_out_name%.o" del /f /q "%path_out_name%.o" >nul 2>&1
if exist "%path_out_name%.bin" del /f /q "%path_out_name%.bin" >nul 2>&1
if exist "%path_out_name%.elf" del /f /q "%path_out_name%.elf" >nul 2>&1
if exist "%path_out_name%.map" del /f /q "%path_out_name%.map" >nul 2>&1

echo [INFO] Compiling...
"%TOOLCHAIN%\arm-none-eabi-gcc.exe" -I"%path_lib%" -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -fsigned-char -Os -w -gdwarf-2 -ffunction-sections -DF_CPU=480000000UL -std=gnu11 -c "%path_src%" -o "%path_out_name%.o"

if %errorlevel% neq 0 (
    echo [ERROR] GCC Compilation failed!
    goto ON_ERROR
)

set "objects_rsp=%SCRIPT_DIR%objects.rsp"
if not exist "%objects_rsp%" (
    echo [ERROR] Missing objects response file: %objects_rsp%
    goto ON_ERROR
)

echo [INFO] Linking...
"%TOOLCHAIN%\arm-none-eabi-gcc.exe" -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -specs=nano.specs -L "%path_lib%\o" -T "%linker_script%" -Wl,--gc-sections "-Wl,-Map=%path_out_name%.map,-cref" "@%objects_rsp%" "%path_out_name%.o" -lm -lc -lnosys -o "%path_out_name%.elf"

if %errorlevel% neq 0 (
    echo [ERROR] GCC Linking failed!
    goto ON_ERROR
)

echo [INFO] Creating binary...
"%TOOLCHAIN%\arm-none-eabi-objcopy.exe" -O binary "%path_out_name%.elf" "%path_out_name%.bin"

if %errorlevel% neq 0 (
    echo [ERROR] Binary generation failed!
    goto ON_ERROR
)

echo.
echo [INFO] Looking for E7-RCU disk...
set "SRC_NAME=%~n1"
if "%SRC_NAME%"=="" set "SRC_NAME=2026v1"

set /a TIMEOUT_COUNT=0
:WAIT_DRIVE
set "Upan="
for /f "tokens=2 delims==" %%a in ('wmic LogicalDisk where "DriveType=2 and VolumeName='E7-RCU'" get DeviceID /value 2^>nul') do set "Upan=%%a"

if "%Upan%"=="" (
    if %TIMEOUT_COUNT% GEQ 30 (
        echo [WARN] E7-RCU disk not found. Build completed but firmware was not copied.
        goto END_OK
    )
    set /a TIMEOUT_COUNT+=1
    "%SystemRoot%\System32\ping.exe" 127.0.0.1 -n 2 >nul
    goto WAIT_DRIVE
)

echo [INFO] E7-RCU found on %Upan%
echo [INFO] Copying %path_out_name%.bin to %Upan%\%SRC_NAME%.bin
copy "%path_out_name%.bin" "%Upan%\%SRC_NAME%.bin"

if %errorlevel% neq 0 (
    echo [ERROR] Firmware copy failed!
    goto ON_ERROR
)

:END_OK
echo.
echo [DONE]
popd
endlocal
exit /b 0

:ON_ERROR
echo [ERROR] Build failed!
popd
endlocal
exit /b 1
