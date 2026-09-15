@echo off
setlocal
title FireFighting Drone Portal
cd /d "%~dp0"

set "PY=.python\python.exe"
if not exist "%PY%" (
  echo The one-time installer has not been run in this folder yet.
  echo Double-click Install.bat first, then start this again.
  pause
  exit /b 1
)
%PY% -c "import flask, pymavlink, bleak, serial" >nul 2>&1
if errorlevel 1 (
  echo Some Python packages are missing. Double-click Install.bat again, then start this.
  pause
  exit /b 1
)

echo Cleaning up any old instances...
powershell -NoProfile -Command "Get-CimInstance Win32_Process -Filter \"Name='python.exe'\" | Where-Object { $_.CommandLine -match 'fire_console|bt_bridge' } | ForEach-Object { Stop-Process -Id $_.ProcessId -Force }" >nul 2>&1

echo Starting the Bluetooth bridge (finds the drone over Bluetooth)...
start "Drone Bluetooth Bridge" cmd /k %PY% fc\bt_bridge.py

timeout /t 3 /nobreak >nul
echo Starting the portal...
start "FireFighting Drone Portal" cmd /k %PY% console\fire_console.py

timeout /t 4 /nobreak >nul
echo Opening the portal page...
start http://127.0.0.1:8008

echo.
echo Ground station is up. The two black windows are the bridge and the
echo console (their logs live there). Close them both to shut everything down.
timeout /t 6 >nul
