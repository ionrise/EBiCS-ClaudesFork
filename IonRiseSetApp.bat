@echo off
rem Starts IonRiseSetApp (needs Python 3 with "Add python.exe to PATH").
cd /d "%~dp0"
where pythonw >nul 2>nul
if %errorlevel%==0 (
  start "" pythonw "%~dp0setup_app\ebics_setup.py"
  exit /b
)
where python >nul 2>nul
if %errorlevel%==0 (
  start "" python "%~dp0setup_app\ebics_setup.py"
  exit /b
)
echo Python 3 is needed for IonRiseSetApp.
echo Download it from https://www.python.org/downloads/ and tick "Add python.exe to PATH".
pause
