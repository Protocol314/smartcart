@echo off
setlocal
pushd "%~dp0.."
set "PYTHONUTF8=1"
python -X utf8 scripts\verify-offline.py %*
set "TASK_RESULT=%ERRORLEVEL%"
popd
exit /b %TASK_RESULT%
