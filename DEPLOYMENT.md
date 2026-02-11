# Deployment Guide

## Frontend Deployment (GitHub Pages)

### Prerequisites
- GitHub account
- Repository with the project code
- GitHub Pages enabled

### Steps

1. **Update Configuration**

Edit `docusaurus.config.js`:

```javascript
const config = {
  url: 'https://YOUR_USERNAME.github.io',
  baseUrl: '/YOUR_REPO_NAME/',
  organizationName: 'YOUR_USERNAME',
  projectName: 'YOUR_REPO_NAME',
  // ...
};
```

2. **Enable GitHub Pages**

- Go to repository Settings
- Navigate to Pages section
- Source: Select "GitHub Actions"

3. **Push to Main Branch**

```bash
git add .
git commit -m "Deploy to GitHub Pages"
git push origin main
```

4. **Monitor Deployment**

- Go to Actions tab in your repository
- Watch the deployment workflow
- Once complete, visit your site at `https://YOUR_USERNAME.github.io/YOUR_REPO_NAME/`

## Backend Deployment

### Option 1: Railway (Recommended)

1. **Sign up at railway.app**

2. **Create New Project**
   - Click "New Project"
   - Select "Deploy from GitHub repo"
   - Choose your repository

3. **Configure Environment**
   - Add environment variables:
     - `OPENAI_API_KEY`
     - `QDRANT_URL`
     - `QDRANT_API_KEY`
     - `DATABASE_URL`
     - `JWT_SECRET_KEY`

4. **Deploy**
   - Railway auto-detects Python
   - Deploys from `backend/` directory
   - Provides a public URL

### Option 2: Render

1. **Sign up at render.com**

2. **Create Web Service**
   - New → Web Service
   - Connect GitHub repository
   - Root Directory: `backend`
   - Build Command: `pip install -r requirements.txt`
   - Start Command: `uvicorn main:app --host 0.0.0.0 --port $PORT`

3. **Add Environment Variables**
   - Same as Railway

4. **Deploy**
   - Render builds and deploys automatically

### Option 3: Fly.io

1. **Install Fly CLI**

```bash
curl -L https://fly.io/install.sh | sh
```

2. **Login**

```bash
fly auth login
```

3. **Create fly.toml**

```toml
app = "your-app-name"

[build]
  builder = "paketobuildpacks/builder:base"

[env]
  PORT = "8000"

[[services]]
  internal_port = 8000
  protocol = "tcp"

  [[services.ports]]
    handlers = ["http"]
    port = 80

  [[services.ports]]
    handlers = ["tls", "http"]
    port = 443
```

4. **Deploy**

```bash
cd backend
fly launch
fly secrets set OPENAI_API_KEY=xxx QDRANT_URL=xxx ...
fly deploy
```

## Database Setup (Neon)

1. **Sign up at neon.tech**

2. **Create Project**
   - Click "Create Project"
   - Choose region
   - Note the connection string

3. **Initialize Database**

```bash
# Use the connection string from Neon
export DATABASE_URL="postgresql://user:pass@host/db"

# Run initialization
python -c "from database import init_db; import asyncio; asyncio.run(init_db())"
```

## Vector Database Setup (Qdrant Cloud)

1. **Sign up at cloud.qdrant.io**

2. **Create Cluster**
   - Free tier available
   - Choose region
   - Note URL and API key

3. **Test Connection**

```python
from qdrant_client import QdrantClient

client = QdrantClient(
    url="YOUR_QDRANT_URL",
    api_key="YOUR_API_KEY"
)

print(client.get_collections())
```

## Environment Variables

### Required Variables

```env
# OpenAI
OPENAI_API_KEY=sk-...

# Qdrant
QDRANT_URL=https://xxx.qdrant.io
QDRANT_API_KEY=xxx

# Database
DATABASE_URL=postgresql://user:pass@host/db

# Security
JWT_SECRET_KEY=your-secret-key-min-32-chars

# CORS (for production)
CORS_ORIGINS=https://yourusername.github.io
```

## Post-Deployment

### 1. Update Frontend API URL

Edit `src/components/ChatBot.js`:

```javascript
const API_URL = 'https://your-backend-url.com';
```

### 2. Index Book Content

Run the indexing script to populate Qdrant:

```bash
python backend/index_content.py
```

### 3. Test Everything

- [ ] Frontend loads correctly
- [ ] Authentication works
- [ ] Chatbot responds
- [ ] Personalization works
- [ ] Translation works
- [ ] Selected text Q&A works

## Monitoring

### Frontend
- GitHub Actions for build status
- GitHub Pages for hosting status

### Backend
- Railway/Render dashboard for logs
- Uptime monitoring (UptimeRobot, etc.)

### Database
- Neon dashboard for connection stats
- Qdrant dashboard for vector operations

## Troubleshooting

### Frontend Issues

**Build fails:**
- Check Node.js version (18+)
- Clear cache: `npm run clear`
- Delete `node_modules` and reinstall

**404 on GitHub Pages:**
- Verify `baseUrl` in config
- Check GitHub Pages settings
- Wait a few minutes for DNS propagation

### Backend Issues

**Database connection fails:**
- Verify DATABASE_URL format
- Check Neon dashboard for status
- Ensure IP whitelist (if applicable)

**Qdrant errors:**
- Verify API key and URL
- Check collection exists
- Review Qdrant dashboard

**CORS errors:**
- Add frontend URL to CORS_ORIGINS
- Restart backend after changes

## Security Checklist

- [ ] Change JWT_SECRET_KEY from default
- [ ] Use environment variables, not hardcoded keys
- [ ] Enable HTTPS (automatic on most platforms)
- [ ] Set appropriate CORS origins
- [ ] Use strong database passwords
- [ ] Rotate API keys regularly
- [ ] Monitor for unusual activity

## Scaling Considerations

### Frontend
- GitHub Pages handles traffic well
- Consider CDN for global users
- Optimize images and assets

### Backend
- Start with basic tier
- Monitor response times
- Scale up as needed
- Consider caching for common queries

### Database
- Neon auto-scales
- Monitor connection pool
- Add indexes for performance

## Cost Estimates

### Free Tier
- GitHub Pages: Free
- Neon: Free tier (0.5GB)
- Qdrant Cloud: Free tier (1GB)
- Railway: $5/month credit
- OpenAI: Pay per use (~$10-50/month)

### Total: ~$15-60/month for moderate usage

## Backup Strategy

1. **Code**: Git repository (already backed up)
2. **Database**: Neon automatic backups
3. **Vector DB**: Export collections periodically
4. **Environment**: Keep `.env.example` updated

## Support

If you encounter issues:
1. Check logs in deployment platform
2. Review this guide
3. Check platform-specific documentation
4. Open GitHub issue with details

---

**Congratulations on deploying your Physical AI book! 🎉**
