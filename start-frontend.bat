@echo off
echo ========================================
echo Starting Physical AI Book Frontend
echo ========================================
echo.
echo Stopping any existing Node processes...
taskkill /F /IM node.exe 2>nul
timeout /t 2 /nobreak >nul
echo.
echo Starting development server...
echo Frontend will be available at: http://localhost:3000
echo.
npm start
