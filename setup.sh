#!/bin/bash

echo "🤖 Physical AI & Humanoid Robotics Book - Setup Script"
echo "======================================================="

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Check Node.js
echo -e "\n${BLUE}Checking Node.js...${NC}"
if command -v node &> /dev/null; then
    echo -e "${GREEN}✓ Node.js $(node --version) found${NC}"
else
    echo -e "${RED}✗ Node.js not found. Please install Node.js 18+${NC}"
    exit 1
fi

# Check Python
echo -e "\n${BLUE}Checking Python...${NC}"
if command -v python3 &> /dev/null; then
    echo -e "${GREEN}✓ Python $(python3 --version) found${NC}"
else
    echo -e "${RED}✗ Python not found. Please install Python 3.10+${NC}"
    exit 1
fi

# Install frontend dependencies
echo -e "\n${BLUE}Installing frontend dependencies...${NC}"
npm install
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Frontend dependencies installed${NC}"
else
    echo -e "${RED}✗ Failed to install frontend dependencies${NC}"
    exit 1
fi

# Setup backend
echo -e "\n${BLUE}Setting up backend...${NC}"
cd backend

# Create virtual environment
if [ ! -d "venv" ]; then
    echo "Creating Python virtual environment..."
    python3 -m venv venv
fi

# Activate virtual environment
source venv/bin/activate

# Install Python dependencies
echo "Installing Python dependencies..."
pip install --upgrade pip
pip install -r requirements.txt

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Backend dependencies installed${NC}"
else
    echo -e "${RED}✗ Failed to install backend dependencies${NC}"
    exit 1
fi

# Create .env if it doesn't exist
if [ ! -f ".env" ]; then
    echo -e "\n${BLUE}Creating .env file...${NC}"
    cp .env.example .env
    echo -e "${GREEN}✓ .env file created${NC}"
    echo -e "${RED}⚠ Please edit backend/.env with your API keys${NC}"
fi

cd ..

# Create necessary directories
echo -e "\n${BLUE}Creating directories...${NC}"
mkdir -p static/img
mkdir -p docs/module3
mkdir -p docs/module4

echo -e "\n${GREEN}========================================${NC}"
echo -e "${GREEN}✓ Setup complete!${NC}"
echo -e "${GREEN}========================================${NC}"

echo -e "\n${BLUE}Next steps:${NC}"
echo "1. Edit backend/.env with your API keys:"
echo "   - OPENAI_API_KEY"
echo "   - QDRANT_URL and QDRANT_API_KEY"
echo "   - DATABASE_URL (Neon Postgres)"
echo "   - JWT_SECRET_KEY"
echo ""
echo "2. Start the frontend:"
echo "   npm start"
echo ""
echo "3. Start the backend (in another terminal):"
echo "   cd backend"
echo "   source venv/bin/activate"
echo "   uvicorn main:app --reload"
echo ""
echo "4. Visit http://localhost:3000"
echo ""
echo -e "${BLUE}For deployment to GitHub Pages:${NC}"
echo "1. Update docusaurus.config.js with your GitHub username"
echo "2. Enable GitHub Pages in repository settings"
echo "3. Push to main branch"
echo ""
echo -e "${GREEN}Happy learning! 🚀${NC}"
