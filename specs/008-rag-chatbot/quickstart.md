# Quickstart Guide: RAG Chatbot

**Feature**: 008-rag-chatbot  
**Last Updated**: 2025-12-11

## Overview

This guide helps developers set up the RAG chatbot development environment and understand the architecture.

---

## Prerequisites

- **Python**: 3.11 or higher
- **Node.js**: 18 or higher (for ChatKit frontend)
- **Git**: For cloning and version control
- **Better Auth**: Already configured in `api-server/`
- **Environment variables**: `.env` file with required credentials

---

## Environment Setup

### 1. Install `uv` (Python Package Manager)

```bash
# macOS/Linux
curl -LsSf https://astral.sh/uv/install.sh | sh

# Windows
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"

# Verify installation
uv --version
```

### 2. Required Environment Variables

Create `.env` file in project root (or update existing):

```env
# OpenAI API (for embeddings and chat completions)
OPENAI_API_KEY=sk-...

# Qdrant Cloud (vector database)
QDRANT_URL=https://your-cluster-name.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key

# Neon Postgres (already configured for Better Auth)
DATABASE_URL=postgresql://user:password@host.neon.tech/dbname

# Better Auth (already configured)
# No additional env vars needed for chatbot
```

**Getting credentials**:

1. **OpenAI API Key**: https://platform.openai.com/api-keys
2. **Qdrant Cloud**: 
   - Sign up at https://qdrant.tech/
   - Create a free cluster
   - Get URL and API key from dashboard
3. **Neon Postgres**: Already configured (existing `DATABASE_URL`)

---

## Project Structure

```
hackathon-book/
├── chatbot/
│   ├── backend/                 # FastAPI + Python
│   │   ├── pyproject.toml      # uv dependencies
│   │   ├── .python-version     # Python 3.11+
│   │   ├── src/
│   │   │   ├── main.py         # FastAPI app entrypoint
│   │   │   ├── api/            # API routes
│   │   │   │   ├── chat.py
│   │   │   │   ├── health.py
│   │   │   │   └── user.py
│   │   │   ├── agents/         # OpenAI Agent definitions
│   │   │   │   ├── triage.py
│   │   │   │   ├── book_qa.py
│   │   │   │   └── selected_text.py
│   │   │   ├── services/       # External service clients
│   │   │   │   ├── qdrant.py
│   │   │   │   ├── neon.py
│   │   │   │   └── openai_client.py
│   │   │   ├── models/         # Pydantic models
│   │   │   │   ├── chat.py
│   │   │   │   ├── user.py
│   │   │   │   └── session.py
│   │   │   ├── utils/          # Helper functions
│   │   │   │   ├── chunking.py
│   │   │   │   └── indexing.py
│   │   │   └── config.py       # Configuration
│   │   ├── scripts/
│   │   │   └── index_book.py   # One-time indexing script
│   │   └── tests/
│   │
│   └── frontend/                # ChatKit + React
│       ├── package.json
│       ├── tsconfig.json
│       ├── src/
│       │   ├── index.tsx       # Main component
│       │   ├── components/
│       │   │   ├── ChatWidget.tsx
│       │   │   └── SettingsPanel.tsx
│       │   └── hooks/
│       │       └── useOpenAiGlobal.ts
│       └── dist/               # Build output
│           └── component.js
│
├── docs/                        # Book content (MDX files)
└── .env                         # Environment variables
```

---

## Backend Setup

### 1. Navigate to Backend Directory

```bash
cd chatbot/backend
```

### 2. Initialize Python Environment with `uv`

```bash
# Create virtual environment
uv venv

# Activate (platform-specific)
# Linux/macOS:
source .venv/bin/activate
# Windows:
.venv\Scripts\activate

# Install dependencies
uv sync
```

### 3. Configure `pyproject.toml`

The `uv sync` command will install from `pyproject.toml`:

```toml
[project]
name = "rag-chatbot-backend"
version = "0.1.0"
description = "RAG Chatbot Backend with FastAPI"
requires-python = ">=3.11"
dependencies = [
    "fastapi>=0.109.0",
    "uvicorn[standard]>=0.27.0",
    "pydantic>=2.6.0",
    "python-dotenv>=1.0.0",
    "openai>=1.10.0",
    "qdrant-client>=1.7.0",
    "asyncpg>=0.29.0",
    "python-multipart>=0.0.6",
]

[project.optional-dependencies]
dev = [
    "pytest>=8.0.0",
    "pytest-asyncio>=0.23.0",
    "ruff>=0.2.0",
    "mypy>=1.8.0",
]

[build-system]
requires = ["hatchling"]
build-backend = "hatchling.build"
```

### 4. Run the Backend Server

```bash
# Development mode with auto-reload
uv run uvicorn src.main:app --reload --host 0.0.0.0 --port 8000

# The API will be available at:
# - http://localhost:8000
# - Swagger docs: http://localhost:8000/docs
# - ReDoc: http://localhost:8000/redoc
```

---

## Frontend Setup

### 1. Navigate to Frontend Directory

```bash
cd chatbot/frontend
```

### 2. Install Dependencies

```bash
npm install
```

### 3. Configure `package.json`

```json
{
  "name": "rag-chatbot-frontend",
  "version": "0.1.0",
  "private": true,
  "scripts": {
    "build": "esbuild src/index.tsx --bundle --format=esm --outfile=dist/component.js",
    "watch": "esbuild src/index.tsx --bundle --format=esm --outfile=dist/component.js --watch",
    "type-check": "tsc --noEmit"
  },
  "dependencies": {
    "react": "^18.2.0",
    "react-dom": "^18.2.0"
  },
  "devDependencies": {
    "@types/react": "^18.2.0",
    "@types/react-dom": "^18.2.0",
    "esbuild": "^0.20.0",
    "typescript": "^5.3.0"
  }
}
```

### 4. Build the Frontend

```bash
# One-time build
npm run build

# Watch mode (auto-rebuild on changes)
npm run watch
```

Output: `dist/component.js` (embedded in FastAPI responses)

---

## Database Setup

### 1. Ensure `expertiseLevel` Column Exists

The `users` table should already have this column from Better Auth setup. Verify:

```sql
-- Connect to Neon Postgres
psql $DATABASE_URL

-- Check column exists
\d users

-- If missing, add it:
ALTER TABLE users 
ADD COLUMN IF NOT EXISTS expertiseLevel VARCHAR(20) 
DEFAULT 'beginner' 
CHECK (expertiseLevel IN ('beginner', 'intermediate', 'advanced'));
```

---

## Qdrant Setup

### 1. Create Collection

Run once to initialize the vector database:

```bash
cd chatbot/backend
uv run python scripts/setup_qdrant.py
```

Or manually via API:

```python
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams

client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

client.recreate_collection(
    collection_name="book_content",
    vectors_config=VectorParams(size=1536, distance=Distance.COSINE)
)
```

### 2. Index Book Content

One-time indexing of MDX files:

```bash
cd chatbot/backend
uv run python scripts/index_book.py --docs-dir ../../docs
```

This script:
1. Parses all MDX files in `docs/`
2. Chunks content by section/subsection
3. Generates embeddings with `text-embedding-3-small`
4. Uploads to Qdrant with metadata

**Re-indexing**: Re-run when book content changes.

---

## Testing the API

### 1. Health Check

```bash
curl http://localhost:8000/api/health
```

Expected response:
```json
{
  "status": "healthy",
  "services": {
    "database": "healthy",
    "vector_db": "healthy",
    "ai_service": "healthy"
  },
  "timestamp": "2025-12-11T10:30:00Z"
}
```

### 2. Chat Endpoint (Requires Auth Token)

```bash
# Get auth token from Better Auth (via frontend login)
TOKEN="your_better_auth_token"

curl -X POST http://localhost:8000/api/chat \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is a Vision-Language-Action model?",
    "selected_text": null,
    "session_id": "sess_abc123"
  }'
```

Expected response:
```json
{
  "answer": "A Vision-Language-Action (VLA) model...",
  "citations": [
    {
      "chapter": "Chapter 4",
      "section": "4.1 Introduction to VLA",
      "line_range": "12-25",
      "file_path": "docs/vla/introduction.mdx"
    }
  ],
  "agent_used": "Book Q&A Agent",
  "confidence": 0.92,
  "chunks_retrieved": 5
}
```

---

## Development Workflow

### 1. Backend Development

```bash
cd chatbot/backend

# Run tests
uv run pytest

# Type checking
uv run mypy src/

# Linting
uv run ruff check src/

# Format code
uv run ruff format src/
```

### 2. Frontend Development

```bash
cd chatbot/frontend

# Watch mode (auto-rebuild)
npm run watch

# Type checking
npm run type-check
```

### 3. Integration Testing

1. Start backend: `uv run uvicorn src.main:app --reload`
2. Start frontend watch: `npm run watch`
3. Open browser to Docusaurus site
4. Test chatbot widget

---

## Common Commands

### Backend (with `uv`)

```bash
# Add new dependency
uv add package-name

# Add dev dependency
uv add --dev pytest

# Remove dependency
uv remove package-name

# Update dependencies
uv sync

# Run Python script
uv run python script.py

# Run tests
uv run pytest

# Start server
uv run uvicorn src.main:app --reload
```

### Frontend (with `npm`)

```bash
# Install dependencies
npm install

# Build once
npm run build

# Watch mode
npm run watch

# Type check
npm run type-check
```

---

## Architecture Overview

### Request Flow

```
User (Docusaurus) 
  → ChatKit Widget (React)
  → POST /api/chat (FastAPI)
  → Authenticate (Better Auth)
  → Triage Agent (OpenAI Agents SDK)
  → Semantic Search (Qdrant)
  → Generate Response (OpenAI GPT-4)
  → Format Citations
  → Return JSON Response
  → Display in ChatKit
```

### Agent Architecture

```
Triage Agent
  ├─→ Book Q&A Agent (general questions)
  │     └─→ search_book_content tool
  │           └─→ Qdrant hybrid search
  │
  └─→ Selected Text Agent (text-specific questions)
        └─→ analyze_selection tool
              └─→ Prioritize selected text context
```

---

## Troubleshooting

### Issue: "Module not found" errors

**Solution**: Ensure virtual environment is activated and dependencies are installed:
```bash
uv sync
source .venv/bin/activate  # or .venv\Scripts\activate on Windows
```

### Issue: Qdrant connection errors

**Solution**: Verify environment variables:
```bash
echo $QDRANT_URL
echo $QDRANT_API_KEY
```

Check Qdrant dashboard for cluster status.

### Issue: OpenAI rate limits

**Solution**: Rate limits are handled automatically with retry logic. Increase delay between requests or upgrade API tier.

### Issue: Better Auth session invalid

**Solution**: Re-authenticate via frontend. Session expires after 30 days of inactivity.

---

## Next Steps

1. ✅ Environment configured
2. → Implement FastAPI backend endpoints
3. → Implement OpenAI Agents
4. → Build ChatKit frontend
5. → Integrate with Docusaurus
6. → Test end-to-end flow

---

## Resources

- **FastAPI Docs**: https://fastapi.tiangolo.com/
- **uv Docs**: https://github.com/astral-sh/uv
- **OpenAI Agents SDK**: https://openai.github.io/openai-agents-python/
- **ChatKit**: https://developers.openai.com/apps-sdk/build/chatgpt-ui
- **Qdrant**: https://qdrant.tech/documentation/
- **Neon**: https://neon.tech/docs/introduction
