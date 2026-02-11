@echo off
echo ========================================
echo Testing Physical AI Book Setup
echo ========================================
echo.
echo Cleaning cache...
call npm run clear
echo.
echo Building site...
call npm run build
echo.
echo If build succeeds, the site is working!
echo.
pause
