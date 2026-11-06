@echo off
setlocal EnableExtensions DisableDelayedExpansion
title RCRC Control Laboratory

rem Start from this script's folder, even when double-clicked or on a UNC share.
pushd "%~dp0"
if errorlevel 1 goto location_failed

set "RCRC_PY=.venv\Scripts\python.exe"
if exist "%RCRC_PY%" goto check_environment
if exist ".venv" goto invalid_environment

echo Preparing a local Python environment for the RCRC desktop app...
py -3 -c "import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)" >nul 2>&1
if not errorlevel 1 goto create_with_launcher
python -c "import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)" >nul 2>&1
if not errorlevel 1 goto create_with_python

echo ERROR: Python 3.10 or newer could not be found.
echo Install Python with Tcl/Tk support and the Python launcher or Add to PATH.
echo Download: https://www.python.org/downloads/windows/
goto failed

:create_with_launcher
py -3 -m venv ".venv"
if errorlevel 1 goto setup_failed
goto check_environment

:create_with_python
python -m venv ".venv"
if errorlevel 1 goto setup_failed

:check_environment
"%RCRC_PY%" -c "import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)" >nul 2>&1
if errorlevel 1 goto invalid_environment
"%RCRC_PY%" -c "import tkinter" >nul 2>&1
if errorlevel 1 goto missing_tk

echo Checking desktop dependencies. First launch needs an Internet connection...
"%RCRC_PY%" -m pip install --disable-pip-version-check --quiet -r "desktop\requirements.txt"
if errorlevel 1 goto setup_failed

echo Starting RCRC Control Laboratory...
"%RCRC_PY%" -m desktop.app %*
set "RCRC_EXIT=%ERRORLEVEL%"
if not "%RCRC_EXIT%"=="0" pause
popd
endlocal & exit /b %RCRC_EXIT%

:invalid_environment
echo ERROR: The existing .venv does not contain a working Windows Python 3.10 or newer.
echo Rename the .venv folder to preserve it, then run this launcher again.
goto failed

:missing_tk
echo ERROR: Tkinter is missing from this Python installation.
echo Modify your Python installation and enable Tcl/Tk and IDLE, then try again.
goto failed

:setup_failed
echo.
echo ERROR: Python environment or dependency setup failed. See the error above.
echo Check your Python installation and Internet connection, then try again.
goto failed

:failed
echo.
pause
popd
endlocal & exit /b 1

:location_failed
echo ERROR: Cannot open the folder containing run_desktop.bat.
pause
endlocal & exit /b 1
