@echo off
cd /d "%~dp0\.."
echo SYNTHETIC DEMO - NOT LIVE LIDAR DATA.
echo Open http://127.0.0.1:8765 in your browser. Ctrl+C to stop.
py -3 radar.py serve --mode demo
pause
