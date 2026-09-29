@echo off
setlocal enabledelayedexpansion

REM Run from the windows\ folder or from the repo root; always resolve paths
REM relative to this script so double-clicking works either way.
set "SCRIPT_DIR=%~dp0"
set "REPO_ROOT=%SCRIPT_DIR%.."
set "VENV_DIR=%REPO_ROOT%\.venv"

echo Looking for Python...

set "PY_CMD="
where py >nul 2>nul
if %errorlevel%==0 (
    py -3 --version >nul 2>nul
    if !errorlevel!==0 set "PY_CMD=py -3"
)

if not defined PY_CMD (
    where python >nul 2>nul
    if !errorlevel!==0 (
        python --version >nul 2>nul
        if !errorlevel!==0 set "PY_CMD=python"
    )
)

if not defined PY_CMD (
    echo ERROR: Python was not found on PATH. Install Python 3 from https://www.python.org/downloads/
    echo and make sure to check "Add python.exe to PATH" during install.
    exit /b 1
)

echo Using: %PY_CMD%

if exist "%VENV_DIR%\Scripts\python.exe" (
    echo Virtual environment already exists at "%VENV_DIR%".
) else (
    echo Creating virtual environment at "%VENV_DIR%"...
    %PY_CMD% -m venv "%VENV_DIR%"
    if errorlevel 1 (
        echo ERROR: Failed to create virtual environment.
        exit /b 1
    )
)

echo Upgrading pip...
"%VENV_DIR%\Scripts\python.exe" -m pip install --upgrade pip

if exist "%REPO_ROOT%\requirements.txt" (
    echo Installing dependencies from requirements.txt...
    "%VENV_DIR%\Scripts\python.exe" -m pip install -r "%REPO_ROOT%\requirements.txt"
    if errorlevel 1 (
        echo WARNING: Some dependencies failed to install. Check the output above.
    )
) else (
    echo WARNING: requirements.txt not found at "%REPO_ROOT%".
)

echo.
echo Setup complete. Use run.bat to start the app.
endlocal
