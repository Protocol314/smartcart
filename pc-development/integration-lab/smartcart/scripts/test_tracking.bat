@echo off
setlocal
pushd "%~dp0.."
set "PYTHONUTF8=1"
set "TASK_REPO=%CD%\.."
set "TASK_CC=%TASK_REPO%\tests\build\toolchains\w64devkit\bin\gcc.exe"
if not exist "%TASK_CC%" set "TASK_CC=gcc"
if not exist "%TASK_REPO%\build\verification\target-tracking\tmp" mkdir "%TASK_REPO%\build\verification\target-tracking\tmp"
set "TEMP=%TASK_REPO%\build\verification\target-tracking\tmp"
set "TMP=%TEMP%"
python -X utf8 scripts\build_uwb_native.py --cc "%TASK_CC%"
if errorlevel 1 goto failed
python -X utf8 -m unittest discover -s tests -p "test_target*.py" -v
if errorlevel 1 goto failed
python -X utf8 -m unittest discover -s tests -p "test_tracking*.py" -v
if errorlevel 1 goto failed
where node >nul 2>nul
if errorlevel 1 goto replay
node tests\test_tracking_frontend.cjs
if errorlevel 1 goto failed
node tests\test_field_preview.cjs
if errorlevel 1 goto failed
:replay
python -X utf8 -m smartcart_tracking --config config\dual_uwb.d5.example.json --uwb-estimation-config config\uwb_estimation.synthetic.json --tracking-config config\target_tracking.synthetic.json --input samples\target_tracking.synthetic.jsonl --summary
if errorlevel 1 goto failed
popd
exit /b 0
:failed
popd
exit /b 1
