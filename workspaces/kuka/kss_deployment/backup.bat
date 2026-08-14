@echo off
setlocal enabledelayedexpansion

REM ============================================
REM KUKA b_ctrldbox Backup Script
REM Copies the CURRENT on-controller files that
REM deploy.bat would overwrite into a timestamped
REM backup folder, for manual comparison later.
REM Controller-side paths are the same regardless
REM of KSS version, so no version selection is
REM needed here.
REM ============================================

echo.
echo ============================================
echo KUKA b_ctrldbox Backup
echo ============================================
echo.

REM ============================================
REM PATH CONFIGURATION - must match deploy.bat
REM ============================================
set "DST_RSI_CONFIG=C:\KRC\ROBOTER\Config\User\Common\SensorInterface"
set "DST_EKI_CONFIG=C:\KRC\ROBOTER\Config\User\Common\EthernetKRL"
set "DST_PROGRAM=C:\KRC\ROBOTER\KRC\R1\Program\b_ctrldbox"

REM ============================================
REM TIMESTAMP
REM ============================================
for /f "skip=1" %%x in ('wmic os get localdatetime') do if not defined MyDate set "MyDate=%%x"
set "TIMESTAMP=%MyDate:~0,8%_%MyDate:~8,6%"

set "BACKUP_ROOT=%~dp0Backup\%TIMESTAMP%"

echo Backup folder:
echo   %BACKUP_ROOT%
echo.

call :BackupFolder "RSI config files" "%DST_RSI_CONFIG%" "%BACKUP_ROOT%\Config\User\Common\SensorInterface"
call :BackupFolder "EKI config files" "%DST_EKI_CONFIG%" "%BACKUP_ROOT%\Config\User\Common\EthernetKRL"
call :BackupFolder "Program files (RSI+EKI)" "%DST_PROGRAM%" "%BACKUP_ROOT%\KRC\R1\Program\b_ctrldbox"

echo.
echo ============================================
echo Backup finished.
echo Backed up files are in:
echo   %BACKUP_ROOT%
echo ============================================
pause
goto :eof

REM ============================================
REM FUNCTION: BackupFolder
REM Arguments: %1=Description, %2=Source (current on-controller), %3=Destination (backup)
REM ============================================
:BackupFolder
set "desc=%~1"
set "src=%~2"
set "dst=%~3"

echo.
echo ============================================
echo BACKUP: %desc%
echo FROM: %src%
echo TO:   %dst%
echo ============================================

if not exist "%src%\*" (
    echo Source folder does not exist or is empty - nothing to back up.
    goto :eof
)

if not exist "%dst%\" mkdir "%dst%"
xcopy /Y /E /I "%src%\*" "%dst%\"
echo Backup completed.
goto :eof
