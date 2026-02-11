# Quick Start Guide

Get your Physical AI book running in 5 minutes!

## Prerequisites

- Node.js 18+ ([Download](https://nodejs.org/))
- Python 3.10+ ([Download](https://www.python.org/))
- Git ([Download](https://git-scm.com/))

## Step 1: Clone Repository

```bash
git clone https://github.com/yourusername/physical-ai-book.git
cd physical-ai-book
```

## Step 2: Run Setup Script

### On Linux/Mac:
```bash
chmod +x setup.sh
./setup.sh
```

### On Windows:
```bash
# Install frontend
npm install

# Setup backend
cd backend
python -m venv venv
venv\Scripts\activate
pip install -r requirements.txt
copy .env.example .env
cd ..
```

## Step 3: Configure Environment

Edit `backend/.env`:

```env
# Get free API keys:
# OpenAI: https://platform.openai.com/api-keys
# Qdrant: https://cloud.qdrant.io/
# Neon: https://neon.tech/

OPENAI_API_KEY=sk-your-key-here
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-key
DATABASE_URL=postgresql://user:pass@host/db
JWT_SECRET_KEY=your-secret-min-32-characters
```

## Step 4: Start Development Servers

### Terminal 1 - Frontend:
```bash
npm start
```

### Terminal 2 - Backend:
```bash
cd backend
source venv/bin/activate  # On Windows: venv\Scripts\activate
uvicorn main:app --reload
```

## Step 5: Open Browser

Visit: http://localhost:3000

## Step 6: Index Content (First Time Only)

```bash
cd backend
python index_content.py
```

## 🎉 You're Ready!

Try these features:
1. Browse the course content
2. Click the chat button (💬)
3. Ask questions about the content
4. Select text and ask about it
5. Sign up to try personalization
6. Click translate button for Urdu

## Common Issues

### Port Already in Use
```bash
# Frontend (port 3000)
npm start -- --port 3001

# Backend (port 8000)
uvicorn main:app --reload --port 8001
```

### Module Not Found
```bash
# Reinstall dependencies
npm install
cd backend && pip install -r requirements.txt
```

### Database Connection Error
- Check DATABASE_URL in .env
- Verify Neon database is running
- Check internet connection

## Next Steps

- Read [README.md](README.md) for full documentation
- Check [DEPLOYMENT.md](DEPLOYMENT.md) for deployment guide
- Review [PROJECT_SUMMARY.md](PROJECT_SUMMARY.md) for architecture

## Need Help?

- Check the documentation
- Open a GitHub issue
- Review error logs in terminal

---

**Happy Learning! 🚀**
