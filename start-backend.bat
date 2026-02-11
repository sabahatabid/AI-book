@echo off
echo Starting Backend Server...
cd backend
call venv\Scripts\activate
echo Backend running on http://localhost:8000
uvicorn main:app --reload --host 0.0.0.0 --port 8000
