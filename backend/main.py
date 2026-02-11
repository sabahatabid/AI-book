from fastapi import FastAPI, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional, List
import os
from dotenv import load_dotenv
import os

# Load .env file from the backend directory
env_path = os.path.join(os.path.dirname(__file__), '.env')
load_dotenv(dotenv_path=env_path)
print(f"Loading .env from: {env_path}")
print(f".env file exists: {os.path.exists(env_path)}")

# Import from local modules (without relative imports)
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

try:
    from auth import auth_router, get_current_user
    print("✓ Successfully imported: auth")
except ImportError as e:
    print(f"✗ Auth import error: {e}")
    auth_router = None
    get_current_user = None

try:
    from database import init_db
    print("✓ Successfully imported: database")
except ImportError as e:
    print(f"✗ Database import error: {e}")
    init_db = None

# Try to import chatbot - Use Mock for now (OpenAI quota issue)
RAGChatbot = None

# Temporarily use Mock chatbot (comment out to try other versions)
try:
    from mock_rag import MockRAGChatbot as RAGChatbot
    print("✓ Using MockRAGChatbot (no API calls needed)")
except ImportError as e:
    print(f"✗ MockRAG import error: {e}")
    RAGChatbot = None

# Uncomment below when you have OpenAI credits:
# try:
#     from rag import RAGChatbot
#     print("✓ Successfully imported: RAGChatbot (with Qdrant)")
# except ImportError as e:
#     print(f"✗ RAG import error: {e}")
#     try:
#         from simple_rag import SimpleRAGChatbot as RAGChatbot
#         print("✓ Successfully imported: SimpleRAGChatbot")
#     except ImportError as e2:
#         print(f"✗ SimpleRAG import error: {e2}")
#         try:
#             from mock_rag import MockRAGChatbot as RAGChatbot
#             print("✓ Successfully imported: MockRAGChatbot")
#         except ImportError as e3:
#             print(f"✗ MockRAG import error: {e3}")
#             RAGChatbot = None

load_dotenv()

# Verify environment variables on startup
print("=" * 50)
print("Environment Variables Check:")
print(f"OPENAI_API_KEY: {'✓ Set' if os.getenv('OPENAI_API_KEY') else '✗ Missing'}")
print(f"QDRANT_URL: {'✓ Set' if os.getenv('QDRANT_URL') else '✗ Missing'}")
print(f"QDRANT_API_KEY: {'✓ Set' if os.getenv('QDRANT_API_KEY') else '✗ Missing'}")
print(f"NEON_DATABASE_URL: {'✓ Set' if os.getenv('NEON_DATABASE_URL') else '✗ Missing'}")
print("=" * 50)

app = FastAPI(
    title="Physical AI & Humanoid Robotics Book API",
    description="Backend API for the Physical AI course with RAG chatbot, personalization, and translation features",
    version="1.0.0",
    docs_url="/docs",
    redoc_url="/redoc"
)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize database
@app.on_event("startup")
async def startup():
    if init_db:
        await init_db()

# Include routers
if auth_router:
    app.include_router(auth_router, prefix="/api/auth", tags=["auth"])

# Initialize RAG chatbot (lazy loading to avoid startup delays)
rag_chatbot = None

def get_rag_chatbot():
    global rag_chatbot
    if rag_chatbot is None and RAGChatbot:
        try:
            print("Attempting to initialize RAG chatbot...")
            rag_chatbot = RAGChatbot()
            print("RAG chatbot initialized successfully!")
        except Exception as e:
            print(f"ERROR: Could not initialize RAG chatbot: {e}")
            import traceback
            traceback.print_exc()
            return None
    return rag_chatbot

class ChatRequest(BaseModel):
    message: str
    selected_text: Optional[str] = None
    chapter: Optional[str] = None

class ChatResponse(BaseModel):
    response: str
    sources: List[dict]

@app.post("/api/chat", response_model=ChatResponse)
async def chat(request: ChatRequest, user=Depends(get_current_user) if get_current_user else None):
    """RAG chatbot endpoint"""
    chatbot = get_rag_chatbot()
    if not chatbot:
        # Fallback response without RAG
        return {
            "response": f"I received your question: '{request.message}'. However, the RAG service is currently unavailable. Please ensure Qdrant is running and configured correctly.",
            "sources": []
        }
    try:
        response = await chatbot.get_response(
            message=request.message,
            selected_text=request.selected_text,
            chapter=request.chapter,
            user_profile=user
        )
        return response
    except Exception as e:
        return {
            "response": f"I'm having trouble processing your request. Error: {str(e)}",
            "sources": []
        }

class PersonalizeRequest(BaseModel):
    content: str
    user_background: dict

@app.post("/api/personalize")
async def personalize_content(request: PersonalizeRequest, user=Depends(get_current_user) if get_current_user else None):
    """Personalize content based on user background"""
    chatbot = get_rag_chatbot()
    if not chatbot:
        return {"personalized_content": request.content}
    try:
        personalized = await chatbot.personalize_content(
            content=request.content,
            user_background=request.user_background
        )
        return {"personalized_content": personalized}
    except Exception as e:
        return {"personalized_content": request.content}

class TranslateRequest(BaseModel):
    content: str
    target_language: str = "ur"

@app.post("/api/translate")
async def translate_content(request: TranslateRequest):
    """Translate content to Urdu"""
    chatbot = get_rag_chatbot()
    if not chatbot:
        return {"translated_content": request.content}
    try:
        translated = await chatbot.translate_content(
            content=request.content,
            target_language=request.target_language
        )
        return {"translated_content": translated}
    except Exception as e:
        return {"translated_content": request.content}

@app.get("/")
async def root():
    """API Homepage"""
    return {
        "message": "Physical AI & Humanoid Robotics Book API",
        "version": "1.0.0",
        "status": "running",
        "endpoints": {
            "docs": "/docs",
            "health": "/api/health",
            "chat": "/api/chat",
            "personalize": "/api/personalize",
            "translate": "/api/translate",
            "debug": "/api/debug"
        },
        "documentation": "Visit /docs for interactive API documentation"
    }

@app.get("/api/debug")
async def debug_info():
    """Debug endpoint to check configuration"""
    chatbot = get_rag_chatbot()
    return {
        "env_loaded": {
            "OPENAI_API_KEY": "Set" if os.getenv("OPENAI_API_KEY") else "Missing",
            "QDRANT_URL": os.getenv("QDRANT_URL") if os.getenv("QDRANT_URL") else "Missing",
            "QDRANT_API_KEY": "Set" if os.getenv("QDRANT_API_KEY") else "Missing",
        },
        "rag_chatbot_status": "Initialized" if chatbot else "Failed to initialize",
        "rag_chatbot_available": RAGChatbot is not None,
        "rag_instance": chatbot is not None
    }

@app.get("/api/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "service": "Physical AI Book API",
        "version": "1.0.0"
    }

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
