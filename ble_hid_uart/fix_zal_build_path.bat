@echo off
setlocal
set ROOT=%~dp0..\..\..\..\..\..
set BUILD=%ROOT%\01_SDK\build
set SHORT=%BUILD%\ble_hid
set LONG=%BUILD%\ble_hid_uart_prj_pan108xxa1_evb

echo ============================================
echo  Fix ZAL Rebuild path (pan1080_Decoder)
echo ============================================
echo.
echo 1) west.cmd redirect is in Toolchain\panchip_bin
echo    (zephyr-env.bat must be used by ZAL - already default)
echo.
echo 2) Junction so ZAL "Open Output Folder" still works:
echo    %LONG%
echo    -^> %SHORT%
echo.

if not exist "%SHORT%\zephyr" (
    echo Short build dir not found. Run rebuild_short.bat first.
    pause
    exit /b 1
)

if exist "%LONG%" (
    echo %LONG% already exists.
    dir "%LONG%" | findstr /i "<JUNCTION>" >nul
    if not errorlevel 1 (
        echo Already a junction. OK.
        goto done
    )
    echo Remove or rename the old folder first, then run this script again.
    echo   (close ZAL / VS Code using that folder)
    pause
    exit /b 1
)

mklink /J "%LONG%" "%SHORT%"
if errorlevel 1 (
    echo Junction failed. Run this window as Administrator once.
    pause
    exit /b 1
)

:done
echo.
echo Done. In ZAL click Rebuild as usual.
echo Output: %SHORT%\zephyr\zephyr.elf
echo.
pause
exit /b 0
