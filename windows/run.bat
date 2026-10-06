@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
set "REPO_ROOT=%SCRIPT_DIR%.."
set "VENV_PY=%REPO_ROOT%\.venv\Scripts\python.exe"

if not exist "%VENV_PY%" (
    echo ERROR: Virtual environment not found at "%REPO_ROOT%\.venv".
    echo Run setup.bat first.
    exit /b 1
)

cd /d "%REPO_ROOT%"
"%VENV_PY%" main.py --interface %*

endlocal
