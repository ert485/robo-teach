@echo off
echo Starting RoboTech Web Controller...
echo.
echo Open your web browser and navigate to:
echo http://localhost:5000
echo.
echo Press Ctrl+C to stop the server
echo.
cd /d "%~dp0"
python app.py
pause