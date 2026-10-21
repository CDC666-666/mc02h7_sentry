@echo off
setlocal
cd /d "%~dp0"

where py >nul 2>nul
if not errorlevel 1 (
  echo Starting full debug page at http://127.0.0.1:8000/index.html?full_debug=1
  start "" "http://127.0.0.1:8000/index.html?full_debug=1"
  py -3 -m http.server 8000
  echo.
  echo The local web server stopped. Read the message above for the reason.
  pause
  exit /b
)

where node >nul 2>nul
if not errorlevel 1 (
  echo Starting full debug page at http://127.0.0.1:8000/index.html?full_debug=1
  start "" "http://127.0.0.1:8000/index.html?full_debug=1"
  node server.js
  echo.
  echo The local web server stopped. Read the message above for the reason.
  pause
  exit /b
)

echo Python 3 or Node.js was not found. A local web server cannot be started.
pause
exit /b 1
