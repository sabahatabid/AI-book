from fastapi import APIRouter, HTTPException, Depends
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from pydantic import BaseModel, EmailStr
from typing import Optional
import os
from datetime import datetime, timedelta
import jwt
from passlib.context import CryptContext

from .database import get_db_connection

auth_router = APIRouter()
security = HTTPBearer()
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

SECRET_KEY = os.getenv("JWT_SECRET_KEY", "your-secret-key-change-in-production")
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 60 * 24 * 7  # 7 days

class SignupRequest(BaseModel):
    email: EmailStr
    password: str
    name: str
    software_background: str
    hardware_background: str
    programming_experience: str
    robotics_experience: str

class LoginRequest(BaseModel):
    email: EmailStr
    password: str

class UserProfile(BaseModel):
    id: int
    email: str
    name: str
    software_background: str
    hardware_background: str
    programming_experience: str
    robotics_experience: str

def create_access_token(data: dict):
    to_encode = data.copy()
    expire = datetime.utcnow() + timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt

def verify_password(plain_password, hashed_password):
    return pwd_context.verify(plain_password, hashed_password)

def get_password_hash(password):
    return pwd_context.hash(password)

async def get_current_user(credentials: HTTPAuthorizationCredentials = Depends(security)):
    try:
        token = credentials.credentials
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        user_id: int = payload.get("sub")
        if user_id is None:
            raise HTTPException(status_code=401, detail="Invalid authentication credentials")
    except jwt.PyJWTError:
        raise HTTPException(status_code=401, detail="Invalid authentication credentials")
    
    conn = await get_db_connection()
    user = await conn.fetchrow("SELECT * FROM users WHERE id = $1", user_id)
    await conn.close()
    
    if user is None:
        raise HTTPException(status_code=401, detail="User not found")
    
    return dict(user)

@auth_router.post("/signup")
async def signup(request: SignupRequest):
    """User signup with background questions"""
    conn = await get_db_connection()
    
    # Check if user exists
    existing_user = await conn.fetchrow("SELECT id FROM users WHERE email = $1", request.email)
    if existing_user:
        await conn.close()
        raise HTTPException(status_code=400, detail="Email already registered")
    
    # Hash password
    hashed_password = get_password_hash(request.password)
    
    # Insert user
    user = await conn.fetchrow("""
        INSERT INTO users (email, password, name, software_background, 
                          hardware_background, programming_experience, robotics_experience)
        VALUES ($1, $2, $3, $4, $5, $6, $7)
        RETURNING id, email, name, software_background, hardware_background, 
                  programming_experience, robotics_experience
    """, request.email, hashed_password, request.name, request.software_background,
        request.hardware_background, request.programming_experience, request.robotics_experience)
    
    await conn.close()
    
    # Create access token
    access_token = create_access_token({"sub": user["id"]})
    
    return {
        "access_token": access_token,
        "token_type": "bearer",
        "user": dict(user)
    }

@auth_router.post("/login")
async def login(request: LoginRequest):
    """User login"""
    conn = await get_db_connection()
    
    user = await conn.fetchrow("SELECT * FROM users WHERE email = $1", request.email)
    await conn.close()
    
    if not user or not verify_password(request.password, user["password"]):
        raise HTTPException(status_code=401, detail="Incorrect email or password")
    
    # Create access token
    access_token = create_access_token({"sub": user["id"]})
    
    return {
        "access_token": access_token,
        "token_type": "bearer",
        "user": {
            "id": user["id"],
            "email": user["email"],
            "name": user["name"],
            "software_background": user["software_background"],
            "hardware_background": user["hardware_background"],
            "programming_experience": user["programming_experience"],
            "robotics_experience": user["robotics_experience"]
        }
    }

@auth_router.get("/me", response_model=UserProfile)
async def get_me(user=Depends(get_current_user)):
    """Get current user profile"""
    return user
