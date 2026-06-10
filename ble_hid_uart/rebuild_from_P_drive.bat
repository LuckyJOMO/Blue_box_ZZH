@echo off
setlocal
if not exist P:\pan1080_Decoder (
    echo ERROR: Run subst first:
    echo   subst P: D:\SW_ApplySpace\BLE_WorkSpace\01_Project_Space\Keya_space\code
    exit /b 1
)

set BUILD_DIR=P:/pan1080_Decoder/01_SDK/build/ble_hid_uart_prj_pan108xxa1_evb
set APP_DIR=P:/pan1080_Decoder/01_SDK/zephyr/samples_panchip/solutions/ble_hid_uart
set BOARD=pan108xxa1_evb

echo BUILD_DIR=%BUILD_DIR%
west build -p always -d %BUILD_DIR% -b %BOARD% %APP_DIR% -- -DCONF_FILE="%APP_DIR%/prj.conf"
exit /b %errorlevel%
