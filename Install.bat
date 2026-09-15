@echo off
setlocal EnableDelayedExpansion
title FireFighting Drone Portal - installer
cd /d "%~dp0"

rem A private copy of Python lives in .python inside this folder. Nothing is installed
rem anywhere else on the laptop and no admin rights are needed. Windows 10 (2018+) or 11, 64-bit.
set "PYVER=3.12.10"
set "PYZIP=python-%PYVER%-embed-amd64.zip"
set "PYURL=https://www.python.org/ftp/python/%PYVER%/%PYZIP%"
set "PYSHA=4acbed6dd1c744b0376e3b1cf57ce906f9dc9e95e68824584c8099a63025a3c3"
set "PIPURL=https://bootstrap.pypa.io/get-pip.py"
set "PYDIR=.python"
set "PY=%PYDIR%\python.exe"

echo.
echo  FireFighting Drone Portal - one-time installer
echo  ==============================================
echo  Puts everything this laptop needs inside this folder. Safe to run again.
echo  Needs an internet connection: about 60 MB the first time.
echo.

rem ---------- 1. Python ----------
echo [1/3] Python %PYVER% ...
if exist "%PY%" (
  echo    already here
) else (
  echo    downloading ...
  call :download "%PYURL%" "%TEMP%\%PYZIP%" || goto :dlfail
  call :checksha "%TEMP%\%PYZIP%" %PYSHA% || goto :shafail
  if exist "%PYDIR%" rmdir /s /q "%PYDIR%"
  mkdir "%PYDIR%"
  call :unzip "%TEMP%\%PYZIP%" "%PYDIR%" || goto :unzipfail
  del "%TEMP%\%PYZIP%" >nul 2>&1
  rem the embeddable build ships with its packages folder switched off; switch it on
  "%PY%" -c "p=r'%PYDIR%\python312._pth';s=open(p).read().replace('#import site','import site');open(p,'w').write(s)"
  echo    installed into %PYDIR%
)
for /f "tokens=*" %%v in ('"%PY%" --version 2^>^&1') do echo    using %%v

rem ---------- 2. packages ----------
echo [2/3] Python packages ...
"%PY%" -m pip --version >nul 2>&1
if errorlevel 1 (
  call :download "%PIPURL%" "%TEMP%\get-pip.py" || goto :dlfail
  "%PY%" "%TEMP%\get-pip.py" --quiet --no-warn-script-location
  del "%TEMP%\get-pip.py" >nul 2>&1
)
"%PY%" -m pip install -r requirements.txt --quiet --disable-pip-version-check --no-warn-script-location
if errorlevel 1 (
  echo    package install FAILED. Check the internet connection and run this again.
  pause
  exit /b 1
)
"%PY%" -c "import flask, pymavlink, bleak, serial, usb, libusb_package" >nul 2>&1
if errorlevel 1 (
  echo    packages installed but do not load. Run this again; if it repeats, open an issue on GitHub.
  pause
  exit /b 1
)
echo    pymavlink, pyserial, flask, bleak, pyusb ready

rem ---------- 3. shortcut ----------
echo [3/3] Desktop shortcut ...
powershell -NoProfile -Command "$s=(New-Object -ComObject WScript.Shell).CreateShortcut([Environment]::GetFolderPath('Desktop')+'\FireFighting Drone Portal.lnk'); $s.TargetPath='%~dp0Start.bat'; $s.WorkingDirectory='%~dp0'; $s.IconLocation='%SystemRoot%\System32\shell32.dll,44'; $s.Save()" >nul 2>&1
echo    "FireFighting Drone Portal" is on the Desktop

echo.
echo  Done. Double-click "FireFighting Drone Portal" on the Desktop to start.
echo  It opens http://127.0.0.1:8008 in your browser.
echo.
echo  Flashing a board for the first time? Windows may need the USB driver for the
echo  board's bootloader once. The Setup page says so when it happens and names the
echo  tool ^(ImpulseRC Driver Fixer^). Betaflight users usually have it already.
echo.
pause
exit /b 0

:dlfail
echo    download failed. Check the internet connection and run this again.
pause
exit /b 1

:shafail
echo    the downloaded file is damaged ^(checksum mismatch^). Run this again.
pause
exit /b 1

:unzipfail
echo    could not unpack Python here. Is this folder writable? Try another folder.
pause
exit /b 1

rem ---------- helpers ----------
:download
rem %1 url, %2 destination. curl ships with Windows 10 since 2018; PowerShell is the fallback.
where curl >nul 2>&1 && ( curl -L -f -sS --retry 3 -o "%~2" "%~1" && exit /b 0 )
powershell -NoProfile -Command "[Net.ServicePointManager]::SecurityProtocol='Tls12'; Invoke-WebRequest -Uri '%~1' -OutFile '%~2' -UseBasicParsing" >nul 2>&1 && exit /b 0
exit /b 1

:checksha
rem %1 file, %2 expected SHA256
set "H="
for /f "skip=1 tokens=1" %%h in ('certutil -hashfile "%~1" SHA256 ^| findstr /v /i "CertUtil"') do if not defined H set "H=%%h"
if /i "!H!"=="%~2" exit /b 0
exit /b 1

:unzip
rem %1 zip, %2 folder. tar ships with Windows 10 since 2018; PowerShell is the fallback.
where tar >nul 2>&1 && ( tar -xf "%~1" -C "%~2" && exit /b 0 )
powershell -NoProfile -Command "Expand-Archive -LiteralPath '%~1' -DestinationPath '%~2' -Force" >nul 2>&1 && exit /b 0
exit /b 1
