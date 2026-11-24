@echo off
setlocal
set "PYTHONUTF8=1"
set "PYTHONDONTWRITEBYTECODE=1"
if "%~1"=="" (
    python "%~dp0..\cart.py" start --mode replay
) else (
    python "%~dp0..\cart.py" %*
)
exit /b %ERRORLEVEL%
