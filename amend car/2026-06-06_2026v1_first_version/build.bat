@echo off
setlocal

REM 获取脚本所在目录
set SCRIPT_DIR=%~dp0
set SCRIPT_DIR=%SCRIPT_DIR:\=\\%
REM 临时将编译器路径加入 PATH 最前面（仅限本次运行）
set "PATH=%~dp0compiler\ARM_v7\bin;%PATH%"
echo %SCRIPT_DIR%
REM 设置库路径（根据你的实际结构调整）
set path_lib=%SCRIPT_DIR%compiler/includeRTOSE7

REM 获取传入的源文件路径
set path_src=%~f1

REM 获取输出文件名（不含扩展名）
set path_out_name=%~dpn1

if exist "%path_out_name%.o" del /f /q "%path_out_name%.o" >nul 2>&1
if exist "%path_out_name%.bin" del /f /q "%path_out_name%.bin" >nul 2>&1
if exist "%path_out_name%.elf" del /f /q "%path_out_name%.elf" >nul 2>&1
if exist "%path_out_name%.map" del /f /q "%path_out_name%.map" >nul 2>&1

REM 设置编译器目录
REM cd "%SCRIPT_DIR%compiler/ARM_v7/bin"

rem cmd0 - 编译阶段
arm-none-eabi-gcc.exe -I%path_lib% -mcpu=cortex-m7 -mthumb  -mfpu=fpv5-d16 -mfloat-abi=hard -fsigned-char -Os -w -gdwarf-2 -ffunction-sections  -DF_CPU=480000000UL -std=gnu11 -c %path_src% -o %path_out_name%.o

REM 【拦截点 1】检查编译是否出错
if %errorlevel% neq 0 (
    echo [ERROR] GCC Compilation failed!
    goto ON_ERROR
)

REM 创建响应文件 objects.rsp（列出所有需要链接的 .o 文件）
set objects_rsp=%SCRIPT_DIR%objects.rsp
REM if exist "%objects_rsp%" del /f /q "%objects_rsp%" >nul 2>&1

REM (
REM echo %path_lib%/o/malloc.o
REM echo %path_lib%/o/piclib.o
REM echo %path_lib%/o/hufftabs.o
REM echo %path_lib%/o/stproc.o
REM echo %path_lib%/o/dqchan.o
REM echo %path_lib%/o/polyphase.o
REM echo %path_lib%/o/dct32.o
REM echo %path_lib%/o/trigtabs.o
REM echo %path_lib%/o/imdct.o
REM echo %path_lib%/o/subband.o
REM echo %path_lib%/o/dequant.o
REM echo %path_lib%/o/huffman.o
REM echo %path_lib%/o/scalfact.o
REM echo %path_lib%/o/bitstream.o
REM echo %path_lib%/o/mp3tabs.o
REM echo %path_lib%/o/buffers.o
REM echo %path_lib%/o/mp3dec.o
REM echo %path_lib%/o/mp3Player.o
REM echo %path_lib%/o/jdy24.o
REM echo %path_lib%/o/exfuns.o
REM echo %path_lib%/o/event_groups.o
REM echo %path_lib%/o/croutine.o
REM echo %path_lib%/o/tjpgd.o
REM echo %path_lib%/o/syscalls.o
REM echo %path_lib%/o/stm32h7xx_hal_sd_ex.o
REM echo %path_lib%/o/timers.o
REM echo %path_lib%/o/es8311.o
REM echo %path_lib%/o/stm32h7xx_hal_exti.o
REM echo %path_lib%/o/stm32h7xx_hal_dma_ex.o
REM echo %path_lib%/o/stm32h7xx_hal_pwr.o
REM echo %path_lib%/o/stm32h7xx_hal_mdma.o
REM echo %path_lib%/o/stm32h7xx_hal_hsem.o
REM echo %path_lib%/o/delay.o
REM echo %path_lib%/o/sysmem.o
REM echo %path_lib%/o/stm32h7xx_hal_i2s.o
REM echo %path_lib%/o/stm32h7xx_hal_tim_ex.o
REM echo %path_lib%/o/stm32h7xx_it.o
REM echo %path_lib%/o/I2C_Soft.o
REM echo %path_lib%/o/lcd_gt911.o
REM echo %path_lib%/o/ccsbcs.o
REM echo %path_lib%/o/diskio.o
REM echo %path_lib%/o/stm32h7xx_ll_fmc.o
REM echo %path_lib%/o/stm32h7xx_hal_sram.o
REM echo %path_lib%/o/stm32h7xx_hal_uart_ex.o
REM echo %path_lib%/o/stm32h7xx_hal_uart.o
REM echo %path_lib%/o/stm32h7xx_hal_pwr_ex.o
REM echo %path_lib%/o/stm32h7xx_hal_pcd_ex.o
REM echo %path_lib%/o/stm32h7xx_ll_usb.o
REM echo %path_lib%/o/stm32h7xx_hal_pcd.o
REM echo %path_lib%/o/usbd_storage_if.o
REM echo %path_lib%/o/usbd_msc.o
REM echo %path_lib%/o/usbd_desc.o
REM echo %path_lib%/o/usbd_conf.o
REM echo %path_lib%/o/usbd_msc_scsi.o
REM echo %path_lib%/o/usbd_msc_data.o
REM echo %path_lib%/o/usbd_msc_bot.o
REM echo %path_lib%/o/usbd_ctlreq.o
REM echo %path_lib%/o/usbd_core.o
REM echo %path_lib%/o/usbd_ioreq.o
REM echo %path_lib%/o/usb_device.o
REM echo %path_lib%/o/stm32h7xx_hal_flash_ex.o
REM echo %path_lib%/o/stm32h7xx_hal_flash.o
REM echo %path_lib%/o/flash_if.o
REM echo %path_lib%/o/stm32h7xx_hal_msp.o
REM echo %path_lib%/o/syscall.o
REM echo %path_lib%/o/ff.o
REM echo %path_lib%/o/file_mgt.o
REM echo %path_lib%/o/freertos.o
REM echo %path_lib%/o/sdmmc.o
REM echo %path_lib%/o/heap_4.o
REM echo %path_lib%/o/list.o
REM echo %path_lib%/o/port.o
REM echo %path_lib%/o/queue.o
REM echo %path_lib%/o/stm32h7xx_ll_sdmmc.o
REM echo %path_lib%/o/stm32h7xx_hal_sd.o
REM echo %path_lib%/o/bsp_driver_sd.o
REM echo %path_lib%/o/cmsis_os.o
REM echo %path_lib%/o/sd_diskio.o
REM echo %path_lib%/o/ff_gen_drv.o
REM echo %path_lib%/o/fatfs.o
REM echo %path_lib%/o/lcd_st7789v.o
REM echo %path_lib%/o/stm32h7xx_hal_cortex.o
REM echo %path_lib%/o/stm32h7xx_hal_dma.o
REM echo %path_lib%/o/stm32h7xx_hal_tim.o
REM echo %path_lib%/o/stm32h7xx_hal_adc_ex.o
REM echo %path_lib%/o/stm32h7xx_hal_rcc_ex.o
REM echo %path_lib%/o/stm32h7xx_hal_rcc.o
REM echo %path_lib%/o/stm32h7xx_hal_adc.o
REM echo %path_lib%/o/startup_stm32h743zitx.o
REM echo %path_lib%/o/system_stm32h7xx.o
REM echo %path_lib%/o/jmkernel.o
REM echo %path_lib%/o/stm32h7xx_hal.o
REM echo %path_lib%/o/tasks.o
REM echo %path_lib%/o/stm32h7xx_hal_gpio.o
REM ) > "%objects_rsp%"

rem cmd1 - 链接阶段（使用响应文件 @objects.rsp）
arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -specs=nano.specs -L %path_lib%/o -T STM32H743ZITX_FLASH.ld -Wl,--gc-sections -Wl,-Map=%path_out_name%.map,-cref @"%objects_rsp%" %path_out_name%.o -lm -lc -lnosys -o %path_out_name%.elf

REM 检查链接是否出错
if %errorlevel% neq 0 (
    echo [ERROR] GCC Linking failed!
    goto ON_ERROR
)

rem cmd2 - 生成 bin 文件
arm-none-eabi-objcopy.exe -O binary %path_out_name%.elf %path_out_name%.bin

REM ========== 拷贝到 E7-RCU 磁盘 ==========
echo.
echo [INFO] Looking for E7-RCU disk...
set "SRC_NAME=%~n1"
echo [INFO] Source file: %SRC_NAME%.c

REM 初始化30秒超时计数器
set /a TIMEOUT_COUNT=0
:WAIT_DRIVE
set Upan=""
for  /f "tokens=2 delims==" %%a in ('wmic LogicalDisk  where "DriveType=2 and VolumeName='E7-RCU'" get DeviceID /value') do set Upan=%%a

IF %Upan%=="" (
    ping 127.0.0.1 -n 2 >nul
    goto WAIT_DRIVE
) 

echo [INFO] E7-RCU found on %E7_DRIVE%
echo [INFO] Copying %path_out_name%.bin to %E7_DRIVE%\%SRC_NAME%.bin
copy "%path_out_name%.bin" "%Upan%\%SRC_NAME%.bin"


:end
echo.
echo [DONE]
endlocal
exit /b 0

:ON_ERROR
echo [ERROR] Build failed!
endlocal
exit /b 1