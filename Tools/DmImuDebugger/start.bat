@echo off
setlocal
cd /d "%~dp0"

py -3 -c "import sys" >nul 2>nul
if not errorlevel 1 (
  echo Starting DM-IMU-L1 debugger at http://127.0.0.1:8001
  start "" "http://127.0.0.1:8001/index.html"
  py -3 -m http.server 8001
  pause
  exit /b
)

python -c "import sys" >nul 2>nul
if not errorlevel 1 (
  echo Starting DM-IMU-L1 debugger at http://127.0.0.1:8001
  start "" "http://127.0.0.1:8001/index.html"
  python -m http.server 8001
  pause
  exit /b
)

node --version >nul 2>nul
if not errorlevel 1 (
  echo Starting DM-IMU-L1 debugger at http://127.0.0.1:8001
  start "" "http://127.0.0.1:8001/index.html"
  node server.js
  pause
  exit /b
)

echo Python 3 and Node.js were not found. Install either one and run this script again.
pause
exit /b 1
