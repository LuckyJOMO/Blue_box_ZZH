@echo off
setlocal

set PAN1080_ROOT=D:\SW_ApplySpace\BLE_WorkSpace\01_Project_Space\Keya_space\code\pan1080_Decoder
set SDK_ROOT=%PAN1080_ROOT%\01_SDK
set TOOLCHAIN_ENV=%PAN1080_ROOT%\05_TOOLS\Toolchain\zephyr-env.bat
rem Short build dir INSIDE pan1080_Decoder (avoid Windows 260-char path limit)
set BUILD_DIR=%SDK_ROOT%/build/ble_hid
set BUILD_DIR=%BUILD_DIR:\=/%
set APP_DIR=%~dp0
set CONF_FILE=%APP_DIR%prj.conf
set CONF_FILE=%CONF_FILE:\=/%
set BOARD=pan108xxa1_evb

if not exist "%TOOLCHAIN_ENV%" (
    echo ERROR: cannot find zephyr-env.bat
    echo   %TOOLCHAIN_ENV%
    pause
    exit /b 1
)
call "%TOOLCHAIN_ENV%"

echo ============================================
echo  pan1080_Decoder short-path build
echo  WEST workspace: %SDK_ROOT%
echo  BUILD_DIR:      %BUILD_DIR%
echo ============================================

cd /d "%SDK_ROOT%"
west build -p always -d %BUILD_DIR% -b %BOARD% %APP_DIR% -- -DCONF_FILE="%CONF_FILE%"
if errorlevel 1 (
    echo.
    echo BUILD FAILED
    pause
    exit /b 1
)

echo.
echo ============================================
echo  BUILD OK
echo  ELF: %BUILD_DIR%/zephyr/zephyr.elf
echo.
echo  In Cursor / ZAL open folder:
echo    %SDK_ROOT%\build\ble_hid
echo ============================================
pause
exit /b 0
