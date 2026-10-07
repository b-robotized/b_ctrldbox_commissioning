@echo off
setlocal enabledelayedexpansion

REM ============================================
REM KUKA b_ctrldbox Deployment Script
REM Supports the three RSI Visual generations used
REM across supported KSS versions:
REM   - RSI 3.3.x (KSS 8.3, 8.4): separate .rsi /
REM                               .rsi.diagram / .rsi.xml files
REM   - RSI 4.0.x (KSS 8.5):      single .rsix file
REM   - RSI 4.1.x (KSS 8.6):      single .rsix file
REM ============================================

REM ============================================
REM RSI VERSION SELECTION
REM ============================================
echo.
echo ============================================
echo KUKA b_ctrldbox Deployment
echo ============================================
echo.
echo Select RSI version (matches your KSS version):
echo   1. RSI 3.3.x - KSS 8.3, 8.4 (separate .rsi / .rsi.diagram / .rsi.xml files)
echo   2. RSI 4.0.x - KSS 8.5 (single .rsix file)
echo   3. RSI 4.1.x - KSS 8.6 (single .rsix file)
echo.
set /p rsi_version="Enter selection (1/2/3): "

if "!rsi_version!"=="1" (
    set "KSS_VERSION_DIR=rsi_3.3.x"
    echo.
    echo Selected: RSI 3.3.x (KSS 8.3, 8.4)
) else if "!rsi_version!"=="3" (
    set "KSS_VERSION_DIR=rsi_4.1.x"
    echo.
    echo Selected: RSI 4.1.x (KSS 8.6)
) else (
    set "KSS_VERSION_DIR=rsi_4.0.x"
    echo.
    echo Selected: RSI 4.0.x (KSS 8.5)
)

REM ============================================
REM RSI CONFIGURATION SELECTION
REM ============================================
echo.
echo Select RSI configuration:
echo   1. Standard (6 robot axes only)
echo   2. External Axis (6 robot axes + external axes support)
echo   3. GPIO (6 robot axes + GPIO support)
echo   4. Extended (Standard + torques, motor currents, program status, setpoint pose) - RSI 4.1.x only
echo   5. Extended + GPIO (Extended + 8 digital inputs / 12 digital outputs) - RSI 4.1.x only
echo.
set /p rsi_config="Enter selection (1/2/3/4/5): "

if "!rsi_config!"=="2" (
    set "RSI_VARIANT=ext_axis"
    echo.
    echo Selected: External Axis configuration
) else if "!rsi_config!"=="3" (
    set "RSI_VARIANT=gpios"
    echo.
    echo Selected: GPIO configuration
) else if "!rsi_config!"=="4" (
    set "RSI_VARIANT=extended"
    echo.
    echo Selected: Extended configuration
) else if "!rsi_config!"=="5" (
    set "RSI_VARIANT=extended_gpios"
    echo.
    echo Selected: Extended + GPIO configuration
) else (
    set "RSI_VARIANT="
    echo.
    echo Selected: Standard configuration
)

REM The Extended variants only exist (and are only tested) for RSI 4.1.x
set "IS_EXTENDED=0"
if "!RSI_VARIANT!"=="extended" set "IS_EXTENDED=1"
if "!RSI_VARIANT!"=="extended_gpios" set "IS_EXTENDED=1"
if "!IS_EXTENDED!"=="1" (
    if not "!KSS_VERSION_DIR!"=="rsi_4.1.x" (
        echo.
        echo ERROR: The Extended configurations are only available for RSI 4.1.x ^(KSS 8.6^).
        echo        Re-run deploy.bat and select RSI version 3.
        pause
        exit /b 1
    )
    echo.
    echo NOTE: Start the driver with rsi_xml_config_file pointing to
    echo       workspaces\kuka\rsi_xml_config\!RSI_VARIANT!.yaml - see RSI_CONFIGURATIONS.md.
)

REM ============================================
REM PATH CONFIGURATION - Update these as needed
REM ============================================
if "!RSI_VARIANT!"=="" (
    set "SRC_RSI_ETH=Config\User\Common\SensorInterface\common"
    set "SRC_RSI_CONFIG=Config\User\Common\SensorInterface\!KSS_VERSION_DIR!"
) else (
    set "SRC_RSI_ETH=Config\User\Common\SensorInterface\common\!RSI_VARIANT!"
    set "SRC_RSI_CONFIG=Config\User\Common\SensorInterface\!KSS_VERSION_DIR!\!RSI_VARIANT!"
)
set "DST_RSI_CONFIG=C:\KRC\ROBOTER\Config\User\Common\SensorInterface"

set "SRC_RSI_PROGRAM=KRC\R1\Program\RSI_kss"

set "SRC_EKI_CONFIG=Config\User\Common\EthernetKRL\kss"
set "DST_EKI_CONFIG=C:\KRC\ROBOTER\Config\User\Common\EthernetKRL"

set "SRC_EKI_PROGRAM=KRC\R1\Program\EKIServer_kss"

set "DST_PROGRAM=C:\KRC\ROBOTER\KRC\R1\Program\b_ctrldbox"

REM ============================================
REM CHECK FOR EXISTING PROGRAM FILES
REM ============================================
set "hasFiles=0"
if exist "%DST_PROGRAM%\*" (
    for %%F in ("%DST_PROGRAM%\*") do set "hasFiles=1"
)
if "!hasFiles!"=="1" (
    echo.
    echo ============================================
    echo WARNING: Program folder contains existing files:
    echo %DST_PROGRAM%
    echo.
    for %%F in ("%DST_PROGRAM%\*") do echo   - %%~nxF
    echo ============================================
    set /p delconfirm="Delete existing program files before deployment? (Y/N): "
    if /i "!delconfirm!"=="Y" (
        del /Q "%DST_PROGRAM%\*"
        echo Existing program files deleted.
    )
)

REM ============================================
REM DEPLOYMENT
REM ============================================
call :CopyFiles "RSI ethernet config files (shared)" "%SRC_RSI_ETH%" "%DST_RSI_CONFIG%"
call :CopyFiles "RSI config files" "%SRC_RSI_CONFIG%" "%DST_RSI_CONFIG%"
call :CopyFiles "RSI program files" "%SRC_RSI_PROGRAM%" "%DST_PROGRAM%"
call :CopyFiles "EKI config files" "%SRC_EKI_CONFIG%" "%DST_EKI_CONFIG%"
call :CopyFiles "EKI program files" "%SRC_EKI_PROGRAM%" "%DST_PROGRAM%"

echo.
echo ============================================
echo Deployment script finished.
echo ============================================
pause
goto :eof

REM ============================================
REM FUNCTION: CopyFiles
REM Arguments: %1=Description, %2=Source, %3=Destination
REM ============================================
:CopyFiles
set "desc=%~1"
set "src=%~2"
set "dst=%~3"

echo.
echo ============================================
echo COPY: %desc%
echo FROM: %src%
echo TO:   %dst%
echo.
echo Files to copy:
for %%F in ("%src%\*") do echo   - %%~nxF
echo ============================================
set /p confirm="Proceed with this copy? (Y/N): "
if /i "!confirm!"=="Y" (
    if not exist "%dst%\" mkdir "%dst%"
    xcopy /Y "%src%\*" "%dst%\"
    echo Copy completed.
) else (
    echo Skipped.
)
goto :eof
