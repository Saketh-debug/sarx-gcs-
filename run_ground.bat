@echo off
set /p COM_PORT="Enter ESP32 COM port (e.g. COM5): "
if "%COM_PORT%"=="" set COM_PORT=COM5
echo Starting Ground Router on %COM_PORT% (115200 baud)...
start "Ground Router" .\.venv\Scripts\python.exe ground\router\ground_router.py --link %COM_PORT% --baud 115200
echo Starting React Web UI...
cd ground\web_ui
npm run dev
pause

