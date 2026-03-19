@echo off
setlocal enabledelayedexpansion

REM ============================================
REM KUKA KSSv8.6 Deployment Script
REM ============================================

REM ============================================
REM RSI CONFIGURATION SELECTION
REM ============================================
echo.
echo ============================================
echo KUKA KSSv8.6 b_ctrldbox Deployment
echo ============================================
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

REM ============================================
REM PATH CONFIGURATION - Update these as needed
REM ============================================
if "!RSI_VARIANT!"=="" (
    set "SRC_RSI_CONFIG=Config\User\Common\SensorInterface"
) else (
    set "SRC_RSI_CONFIG=Config\User\Common\SensorInterface\!RSI_VARIANT!"
)
set "DST_RSI_CONFIG=C:\KRC\ROBOTER\Config\User\Common\SensorInterface"

set "SRC_RSI_PROGRAM=KRC\R1\Program\RSI"

set "SRC_EKI_CONFIG=Config\User\Common\EthernetKRL"
set "DST_EKI_CONFIG=C:\KRC\ROBOTER\Config\User\Common\EthernetKRL"

set "SRC_EKI_PROGRAM=KRC\R1\Program\EKIserver"

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
