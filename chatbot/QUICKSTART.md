# RAG Chatbot - Quick Start Guide

Get the chatbot running in 5 minutes!

## Prerequisites Check

```bash
# Check Python version (need 3.11+)
python --version

# Check Node.js version (need 20+)
node --version

# Check if uv is installed
uv --version
```

If uv is not installed:
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

## Step 1: Environment Variables (2 minutes)

Create `.env` file in **project root** (D:\workspace\nextjs\hackathon-book\):

```env
# Get from https://platform.openai.com/api-keys
OPENAI_API_KEY=sk-...

# Get from https://qdrant.io (free tier available)
QDRANT_URL=https://your-cluster-name.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key

# Should already exist from Better Auth setup
DATABASE_URL=postgresql://user:password@host:5432/dbname

NODE_ENV=development
PORT=8000
```

## Step 2: Backend Setup (2 minutes)

```bash
# Navigate to backend
cd chatbot/backend

# Install dependencies
uv sync

# Ingest book content (one-time setup)
uv run scripts/ingest_book_content.py

# Start backend server
uv run run.py
```

Backend should start at http://localhost:8000

Visit http://localhost:8000/docs to see the API documentation

## Step 3: Frontend Setup (1 minute)

Open a **new terminal**:

```bash
# Navigate to frontend
cd chatbot/frontend

# Install dependencies
npm install

# Start dev server
npm run dev
```

Frontend should start at http://localhost:5173

## Step 4: Test It! 

1. Open http://localhost:5173 in your browser
2. Select your expertise level (Beginner/Intermediate/Advanced)
3. Ask a question like:
   - "What is a transformer model?"
   - "Explain attention mechanisms"
   - "How does RAG work?"

## Troubleshooting

### Backend won't start

**Problem**: `uv: command not found`
```bash
# Use full path (Windows Git Bash)
"C:\Users\YourUsername\.local\bin\uv.exe" sync
```

**Problem**: Database connection error
- Check DATABASE_URL in .env
- Ensure Neon Postgres is accessible

**Problem**: Qdrant connection error
- Verify QDRANT_URL and QDRANT_API_KEY
- Check Qdrant Cloud dashboard

### Frontend won't connect

**Problem**: CORS error
- Ensure backend is running on port 8000
- Check CORS settings in `backend/app/main.py`

**Problem**: Session creation fails
- Check backend logs for errors
- Verify database tables were created

### No AI responses

**Problem**: Empty responses
- Ensure book content was ingested (Step 2)
- Check Qdrant collection has data: http://localhost:8000/docs → Try `/health`

**Problem**: OpenAI API error
- Verify OPENAI_API_KEY is valid
- Check you have API credits

## What's Next?

- Read `chatbot/README.md` for architecture details
- Check `chatbot/backend/README.md` for API documentation
- See `chatbot/frontend/README.md` for frontend customization
- Review `specs/008-rag-chatbot/spec.md` for features roadmap

## Quick Commands Reference

```bash
# Backend
cd chatbot/backend
uv run run.py                              # Start server
uv run scripts/ingest_book_content.py      # Reload book data
uv run pytest                              # Run tests (when added)

# Frontend
cd chatbot/frontend
npm run dev                                # Start dev server
npm run build                              # Build for production
npm run preview                            # Preview production build

# Data Management
# Re-ingest with custom directory
uv run scripts/ingest_book_content.py /path/to/docs
```

## Support

- API Docs: http://localhost:8000/docs
- Backend README: `chatbot/backend/README.md`
- Frontend README: `chatbot/frontend/README.md`
- Main README: `chatbot/README.md`
- Feature Spec: `specs/008-rag-chatbot/spec.md`

Happy Learning! 🚀
