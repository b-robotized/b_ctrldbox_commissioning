@echo off
setlocal enabledelayedexpansion

REM ============================================
REM KUKA Deployment Script
REM Deploys either:
REM   - b_ctrldbox (ROS 2 EKI+RSI driver), or
REM   - the vendored FZI reference driver (RSI-only)
REM
REM b_ctrldbox supports both RSI packaging formats:
REM   - KSS 8.6 and above: single .rsix file
REM   - KSS below 8.6:     separate .rsi /
REM                        .rsi.diagram / .rsi.xml files
REM ============================================

REM ============================================
REM DRIVER SELECTION
REM ============================================
echo.
echo ============================================
echo KUKA Deployment
echo ============================================
echo.
echo Select driver to deploy:
echo   1. b_ctrldbox (ROS 2 EKI+RSI driver)
echo   2. FZI reference driver (kuka_rsi_driver, RSI-only, no ext_axis/GPIO)
echo.
set /p driver_choice="Enter selection (1/2): "

if "!driver_choice!"=="2" (
    set "DRIVER=fzi"
    echo.
    echo Selected: FZI reference driver
) else (
    set "DRIVER=b_ctrldbox"
    echo.
    echo Selected: b_ctrldbox
)

if "!DRIVER!"=="fzi" goto :FziSelection
goto :BCtrldboxSelection

REM ============================================
REM b_ctrldbox: KSS VERSION + RSI CONFIGURATION SELECTION
REM ============================================
:BCtrldboxSelection
echo.
echo Select KSS version:
echo   1. KSS 8.6 and above (single .rsix file)
echo   2. KSS below 8.6 (separate .rsi / .rsi.diagram / .rsi.xml files)
echo.
set /p kss_version="Enter selection (1/2): "

if "!kss_version!"=="2" (
    set "KSS_VERSION_DIR=below_v8.6"
    echo.
    echo Selected: KSS below 8.6
) else (
    set "KSS_VERSION_DIR=above_v8.6"
    echo.
    echo Selected: KSS 8.6 and above
)

echo.
echo Select RSI configuration:
echo   1. Standard (6 robot axes only)
echo   2. External Axis (6 robot axes + external axes support)
echo   3. GPIO (6 robot axes + GPIO support)
echo.
set /p rsi_config="Enter selection (1/2/3): "

if "!rsi_config!"=="2" (
    set "RSI_VARIANT=ext_axis"
    echo.
    echo Selected: External Axis configuration
) else if "!rsi_config!"=="3" (
    set "RSI_VARIANT=gpios"
    echo.
    echo Selected: GPIO configuration
) else (
    set "RSI_VARIANT="
    echo.
    echo Selected: Standard configuration
)

if "!RSI_VARIANT!"=="" (
    set "SRC_RSI_CONFIG=Config\User\Common\SensorInterface\!KSS_VERSION_DIR!"
) else (
    set "SRC_RSI_CONFIG=Config\User\Common\SensorInterface\!KSS_VERSION_DIR!\!RSI_VARIANT!"
)
set "SRC_RSI_PROGRAM=KRC\R1\Program\RSI"
set "SRC_EKI_CONFIG=Config\User\Common\EthernetKRL"
set "SRC_EKI_PROGRAM=KRC\R1\Program\EKIserver"
set "DST_PROGRAM=C:\KRC\ROBOTER\KRC\R1\Program\b_ctrldbox"
goto :PathsDone

REM ============================================
REM FZI: CONTROLLER GENERATION SELECTION
REM ============================================
:FziSelection
echo.
echo Select KUKA controller generation:
echo   1. KR C4 (split .rsi / .rsi.diagram / .rsi.xml files)
echo   2. KR C5 (single .rsix file)
echo.
set /p controller_gen="Enter selection (1/2): "

if "!controller_gen!"=="2" (
    set "FZI_DIR=KR_C5"
    echo.
    echo Selected: KR C5
) else (
    set "FZI_DIR=KR_C4"
    echo.
    echo Selected: KR C4
)

set "SRC_RSI_CONFIG=fzi_reference\Config\User\Common\SensorInterface\!FZI_DIR!"
set "SRC_RSI_PROGRAM=fzi_reference\KRC\R1\Program\FZIdriver\!FZI_DIR!"
set "SRC_EKI_CONFIG="
set "SRC_EKI_PROGRAM="
set "DST_PROGRAM=C:\KRC\ROBOTER\KRC\R1\Program\fzi_rsi_driver"
goto :PathsDone

:PathsDone
REM ============================================
REM PATH CONFIGURATION - Update these as needed
REM ============================================
set "DST_RSI_CONFIG=C:\KRC\ROBOTER\Config\User\Common\SensorInterface"
set "DST_EKI_CONFIG=C:\KRC\ROBOTER\Config\User\Common\EthernetKRL"

REM ============================================
REM CLEAN UP STALE RSI CONFIG FILES
REM DST_RSI_CONFIG is a single shared folder on the
REM controller. Both b_ctrldbox (any KSS version /
REM variant) and the FZI reference driver write into
REM it under different filenames, so switching what
REM you deploy can leave stale files from a previous,
REM different version/variant/driver behind. Delete
REM all known filenames before copying the newly
REM selected set so only one coherent config remains.
REM ============================================
echo.
echo ============================================
echo Cleaning up stale RSI config files in:
echo %DST_RSI_CONFIG%
echo ============================================
set "STALE_CLEANED=0"
for %%F in (
    b_ctrldbox_rsi.rsix
    b_ctrldbox_rsi.rsi
    b_ctrldbox_rsi.rsi.diagram
    b_ctrldbox_rsi.rsi.xml
    b_ctrldbox_rsi_eth.xml
    ros_rsi.rsix
    ros_rsi.rsi
    ros_rsi.rsi.diagram
    ros_rsi.rsi.xml
    ros_rsi_ethernet.xml
) do (
    if exist "%DST_RSI_CONFIG%\%%F" (
        echo   Deleting stale file: %%F
        del /Q "%DST_RSI_CONFIG%\%%F"
        set "STALE_CLEANED=1"
    )
)
if "!STALE_CLEANED!"=="0" (
    echo   No stale RSI config files found.
)

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
call :CopyFiles "RSI config files" "%SRC_RSI_CONFIG%" "%DST_RSI_CONFIG%"
call :CopyFiles "RSI program files" "%SRC_RSI_PROGRAM%" "%DST_PROGRAM%"
if not "!SRC_EKI_CONFIG!"=="" (
    call :CopyFiles "EKI config files" "%SRC_EKI_CONFIG%" "%DST_EKI_CONFIG%"
)
if not "!SRC_EKI_PROGRAM!"=="" (
    call :CopyFiles "EKI program files" "%SRC_EKI_PROGRAM%" "%DST_PROGRAM%"
)

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
