@echo off
setlocal
title Observer01 - Start Work
set "LAUNCH_FAILED="

call :open "D:\keil5\UV4\UV4.exe" "D:\2Electrical machinery\G431_peizhi\Observer01 - hfi\observer01\MDK-ARM\observer01.uvprojx"
call :open "C:\Users\28436\AppData\Local\Programs\Microsoft VS Code\Code.exe" "D:\2Electrical machinery\G431_peizhi\Observer01 - hfi\observer01\MDK-ARM\observer01.code-workspace"
call :background "MotorPilot.exe" "D:\WangyeDownload\MC_SDK_6.4.1\Utilities\PC_Software\STMotorPilot\MotorPilot.exe"

if defined LAUNCH_FAILED pause
exit /b

:background
tasklist /FI "IMAGENAME eq %~1" /NH 2>nul | find /I "%~1" >nul
if not errorlevel 1 exit /b 0
call :open "%~2"
exit /b

:open
if not exist "%~1" (
    echo [ERROR] Program not found: %~1
    set "LAUNCH_FAILED=1"
    exit /b 1
)
if not "%~2"=="" if not exist "%~2" (
    echo [ERROR] Project not found: %~2
    set "LAUNCH_FAILED=1"
    exit /b 1
)
rem Use VS Code's CLI: it detaches the editor and disables console attachment.
if /I "%~nx1"=="Code.exe" (
    if not exist "%~dp1bin\code.cmd" (
        echo [ERROR] VS Code launcher not found: %~dp1bin\code.cmd
        set "LAUNCH_FAILED=1"
        exit /b 1
    )
    call "%~dp1bin\code.cmd" "%~2"
    if errorlevel 1 (
        echo [ERROR] Could not start VS Code.
        set "LAUNCH_FAILED=1"
        exit /b 1
    )
    exit /b 0
)

rem Pass paths through environment variables to preserve spaces and quoting.
set "LAUNCH_EXE=%~1"
set "LAUNCH_DIR=%~dp1"
set "LAUNCH_TARGET=%~2"
rem Do not inherit Electron's Node-only mode when launched from VS Code.
set "ELECTRON_RUN_AS_NODE="
powershell.exe -NoLogo -NoProfile -NonInteractive -Command "$ErrorActionPreference='Stop'; try { $launchOptions=@{FilePath=$env:LAUNCH_EXE;WorkingDirectory=$env:LAUNCH_DIR;WindowStyle='Normal'}; if ($env:LAUNCH_TARGET) { $launchOptions.ArgumentList=[char]34+$env:LAUNCH_TARGET+[char]34 }; Start-Process @launchOptions } catch { Write-Host $_.Exception.Message; exit 1 }"
if errorlevel 1 (
    echo [ERROR] Could not start: %~1
    set "LAUNCH_FAILED=1"
)
exit /b
