@echo off
setlocal
pushd "%~dp0.."
set "PYTHONUTF8=1"
set "TASK_REPO=%CD%\.."
set "TASK_CC=%TASK_REPO%\tests\build\toolchains\w64devkit\bin\gcc.exe"
if not exist "%TASK_CC%" set "TASK_CC=gcc"
if not exist "%TASK_REPO%\build\verification\dual-uwb\tmp" mkdir "%TASK_REPO%\build\verification\dual-uwb\tmp"
set "TEMP=%TASK_REPO%\build\verification\dual-uwb\tmp"
set "TMP=%TEMP%"
python -X utf8 scripts\build_uwb_native.py --cc "%TASK_CC%"
if errorlevel 1 goto failed
python -X utf8 -m unittest discover -s tests -p "test_dual_uwb*.py" -v
if errorlevel 1 goto failed
python -X utf8 -m smartcart_uwb replay --config config\dual_uwb.d5.example.json --input samples\dual_uwb.synthetic.jsonl --summary
if errorlevel 1 goto failed
python -X utf8 -m smartcart_uwb estimate-replay --config config\dual_uwb.d5.example.json --estimation-config config\uwb_estimation.synthetic.json --input samples\dual_uwb.position.synthetic.jsonl --summary
if errorlevel 1 goto failed
popd
exit /b 0
:failed
popd
exit /b 1
