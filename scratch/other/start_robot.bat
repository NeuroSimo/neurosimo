setlocal
set PORT=5000

echo Finding PID for port %PORT%...

for /f "tokens=5" %%i in ('netstat -aon ^| findstr :%PORT%') do (
    set PID=%%i
    goto :killProcess
)

:killProcess
if not defined PID (
    echo No process found running on port %PORT%.
    goto :end
)

echo Found process with PID %PID%, killing...
taskkill /F /PID %PID%
if %ERRORLEVEL% == 0 (
    echo Successfully killed process.
) else (
    echo Failed to kill process with PID %PID%.
)

:end
endlocal

start "relay server" call python C:/Users/mTMS/Robot_TMS/relay_server.py 5000

timeout /t 1 /nobreak

cd /d C:\Users\mTMS\Robot_TMS

start "main loop" call python C:/Users/mTMS/Robot_TMS/main_loop.py
