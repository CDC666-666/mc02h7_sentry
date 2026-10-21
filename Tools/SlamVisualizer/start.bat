@echo off
setlocal
cd /d "%~dp0"

where py >nul 2>nul
if not errorlevel 1 (
  echo Starting SLAM debug web page at http://localhost:8001/index.html?slam_debug=1
  start "" "http://localhost:8001/index.html?slam_debug=1"
  py -3 -m http.server 8001
  echo.
  echo The local web server stopped. Read the message above for the reason.
  pause
  exit /b
)

where node >nul 2>nul
if not errorlevel 1 (
  echo Starting SLAM debug web page at http://localhost:8001/index.html?slam_debug=1
  start "" "http://localhost:8001/index.html?slam_debug=1"
  node server.js
  echo.
  echo The local web server stopped. Read the message above for the reason.
  pause
  exit /b
)

echo Python 3 or Node.js was not found. A local web server cannot be started.
pause
exit /b 1
