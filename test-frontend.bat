@echo off
echo Cleaning Docusaurus cache...
call npm run clear

echo.
echo Starting frontend server...
echo Frontend will be available at http://localhost:3000
echo.
call npm start
