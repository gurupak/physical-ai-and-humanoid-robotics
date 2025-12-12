# Research: RAG Chatbot Technology Stack

**Date**: 2025-12-11  
**Feature**: 008-rag-chatbot

## Executive Summary

This document consolidates research findings for the RAG chatbot implementation using FastAPI, OpenAI Agents SDK (Python), OpenAI ChatKit, Neon Postgres, Qdrant Cloud, and uv package manager.

---

## Technology Decisions

### 1. Python Package Management: `uv`

**Decision**: Use `uv` (by Astral) as the Python package manager and environment manager

**Rationale**:
- **10-100x faster** than pip/pip-tools for package installation and resolution
- Built-in virtual environment management (`uv venv`)
- Single tool for dependency management, similar to npm/pnpm for Node.js
- Native `pyproject.toml` and `requirements.txt` support
- Built in Rust for performance
- Actively maintained by Astral (creators of Ruff)
- Latest stable version available as of December 2024

**Installation**:
```bash
# macOS/Linux
curl -LsSf https://astral.sh/uv/install.sh | sh

# Windows
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"

# Or via pip
pip install uv
```

**Usage patterns**:
```bash
# Initialize project
uv venv

# Add dependencies
uv add fastapi qdrant-client openai

# Add dev dependencies
uv add --dev pytest ruff mypy

# Sync dependencies
uv sync

# Run commands
uv run python script.py
uv run pytest
```

**Alternatives considered**:
- `pip` + `virtualenv`: Standard but slow, lacks integrated dependency resolution
- `poetry`: Good but slower than uv, heavier dependency tree
- `pipenv`: Deprecated, slow lock file generation

---

### 2. Web Framework: FastAPI

**Decision**: Use FastAPI 0.109+ (latest) with async/await patterns for all endpoints

**Rationale**:
- **Native async support** for concurrent request handling
- **Automatic OpenAPI/Swagger documentation** generation
- **Pydantic v2 integration** for request/response validation
- **Excellent performance** (comparable to Node.js/Go)
- **Type hints** for IDE autocompletion and static analysis
- **WebSocket and SSE support** for streaming responses
- **Dependency injection system** for clean separation of concerns

**Best practices identified**:
```python
from fastapi import FastAPI, Depends
from pydantic import BaseModel
import asyncio

app = FastAPI()

# Use async endpoints for I/O operations
@app.post("/chat")
async def chat_endpoint(request: ChatRequest):
    # Async operations don't block the event loop
    result = await process_chat_query(request)
    return result

# Dependency injection for shared resources
async def get_db_session():
    async with db_pool.connection() as conn:
        yield conn

@app.get("/history")
async def get_history(db = Depends(get_db_session)):
    return await db.fetch_all("SELECT * FROM conversations")
```

**Key configuration**:
- Uvicorn ASGI server with `--reload` for development
- CORS middleware for cross-origin requests from Docusaurus frontend
- Structured logging with correlation IDs
- Request timeout middleware (30s default)
- Rate limiting via SlowAPI

**Alternatives considered**:
- Flask: Synchronous by default, less modern async support
- Django: Too heavyweight for API-only backend
- Starlette: FastAPI is built on Starlette, FastAPI adds conveniences

---

### 3. Multi-Agent Orchestration: OpenAI Agents SDK (Python)

**Decision**: Use OpenAI Agents SDK (Python Swarm) for multi-agent orchestration

**Rationale**:
- **Official OpenAI SDK** for agent orchestration
- **Handoff pattern** for agent specialization (Triage → Book Q&A → Selected Text)
- **Tool integration** with function calling
- **Context management** built-in
- **Streaming support** for real-time responses
- **Type-safe** with Pydantic models

**Architecture pattern**:
```python
from agents import Agent, Runner, function_tool

@function_tool
def search_book_content(query: str, expertise_level: str) -> str:
    """Search book content with semantic search"""
    # Dynamic chunk retrieval based on complexity and expertise
    chunks = await qdrant_search(query, expertise_level)
    return format_context(chunks)

# Define specialized agents
triage_agent = Agent(
    name="Triage Agent",
    instructions="Route questions to appropriate specialist",
    handoffs=[book_qa_agent, selected_text_agent]
)

book_qa_agent = Agent(
    name="Book Q&A Agent",
    instructions="Answer questions about book content",
    tools=[search_book_content]
)

selected_text_agent = Agent(
    name="Selected Text Agent",
    instructions="Answer questions about selected text passages",
    tools=[analyze_selection]
)

# Execute with streaming
async def handle_chat(query: str, context: dict):
    result = await Runner.run(
        triage_agent,
        query,
        context=context
    )
    return result.final_output
```

**Key features**:
- **Handoff-based routing**: Triage agent routes to specialists
- **Tool use behavior**: Custom handlers for dynamic retrieval
- **Guardrails**: Input validation before agent processing
- **Context preservation**: Maintains conversation state

**Alternatives considered**:
- LangChain: More complex, heavier abstractions
- Custom implementation: Reinventing the wheel, no streaming support
- AutoGPT/BabyAGI: Not designed for conversational RAG

---

### 4. Frontend Chat UI: OpenAI ChatKit

**Decision**: Use OpenAI ChatKit for chat interface embedded in Docusaurus

**Rationale**:
- **Official OpenAI UI library** optimized for chat interfaces
- **Built-in streaming support** via `window.openai` API
- **Theme integration** (light/dark mode)
- **Responsive design** for mobile/desktop
- **State management** via `widgetState` API
- **Iframe sandboxing** for security
- **React-based** with TypeScript support

**Integration pattern**:
```typescript
// Component reads from window.openai
import { useOpenAiGlobal } from './hooks';

export function ChatWidget() {
  const toolOutput = useOpenAiGlobal('toolOutput');
  const theme = useOpenAiGlobal('theme');
  
  async function sendMessage(message: string) {
    await window.openai.callTool('chat', {
      query: message,
      selectedText: getSelection()
    });
  }
  
  return <ChatInterface messages={toolOutput} theme={theme} />;
}
```

**Key capabilities**:
- `window.openai.callTool()`: Direct MCP tool calls
- `window.openai.sendFollowUpMessage()`: Conversational follow-ups
- `window.openai.setWidgetState()`: Persist user preferences
- `window.openai.requestDisplayMode()`: Inline/fullscreen/PiP layouts

**Deployment**:
- Component bundled with esbuild to `dist/component.js`
- Embedded in MCP server tool responses
- Served from FastAPI backend

**Alternatives considered**:
- Custom React chat UI: More work, no built-in streaming
- LangChain ChatUI: Not optimized for OpenAI Agents SDK
- Third-party chat libraries: No `window.openai` integration

---

### 5. Vector Database: Qdrant Cloud

**Decision**: Use Qdrant Cloud (free tier) for vector search with Python client

**Rationale**:
- **Cloud-hosted**: No infrastructure management
- **Free tier**: 1GB storage, sufficient for book content
- **Fast similarity search** with HNSW indexing
- **Metadata filtering** for section/chapter filtering
- **Hybrid search**: Combines dense vectors + sparse vectors (BM25)
- **Python client** with async support
- **REST API** for flexibility

**Setup**:
```python
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct

# Initialize client
client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

# Create collection
client.recreate_collection(
    collection_name="book_content",
    vectors_config=VectorParams(
        size=1536,  # text-embedding-3-small dimension
        distance=Distance.COSINE
    )
)

# Upsert vectors
points = [
    PointStruct(
        id=chunk_id,
        vector=embedding,
        payload={
            "chapter": "Chapter 3",
            "section": "3.2 Digital Twins",
            "content": chunk_text,
            "line_range": "45-78"
        }
    )
]
client.upsert(collection_name="book_content", points=points)

# Search
results = client.search(
    collection_name="book_content",
    query_vector=query_embedding,
    limit=10,
    score_threshold=0.7
)
```

**Environment variables required**:
```env
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-api-key-here
```

**Indexing strategy**:
- **One-time build script**: Run during deployment to index MDX files
- **Chunking**: Section/subsection boundaries (preserve semantic coherence)
- **Embedding model**: `text-embedding-3-small` (1536 dimensions, cost-effective)
- **Metadata**: chapter, section, line ranges for citations

**Alternatives considered**:
- Pinecone: More expensive, less flexible free tier
- Weaviate: More complex setup, heavier infrastructure
- Chroma: Self-hosted, would require additional infrastructure

---

### 6. Database: Neon Postgres Serverless

**Decision**: Use Neon Postgres with `@neondatabase/serverless` driver

**Rationale**:
- **Already implemented** in the project (Better Auth uses it)
- **Serverless**: Auto-scaling, pay-per-use
- **Low-latency**: Message pipelining optimizations
- **WebSocket support**: For session management
- **Free tier**: Generous limits for development

**Python connection** (for FastAPI backend):
```python
import os
from psycopg2 import pool

# Connection pool
db_pool = pool.SimpleConnectionPool(
    minconn=1,
    maxconn=10,
    dsn=os.getenv("DATABASE_URL")
)

# Or use asyncpg for async operations
import asyncpg

async def get_user_expertise(user_id: str):
    conn = await asyncpg.connect(os.getenv("DATABASE_URL"))
    try:
        row = await conn.fetchrow(
            "SELECT expertiseLevel FROM users WHERE id = $1",
            user_id
        )
        return row['expertiseLevel'] if row else 'beginner'
    finally:
        await conn.close()
```

**Schema requirements**:
- `users.expertiseLevel`: Enum('beginner', 'intermediate', 'advanced')
- Session management handled by Better Auth (already configured)

**Environment variable** (already configured):
```env
DATABASE_URL=postgresql://user:pass@host.neon.tech/dbname
```

**Alternatives considered**:
- SQLite: No concurrent writes, not suitable for multi-user
- MongoDB: Schema-less doesn't fit our relational needs
- Supabase: Similar to Neon but already using Neon

---

## Architectural Decisions

### 1. Project Structure

**Decision**: Separate `chatbot/backend` and `chatbot/frontend` directories

```
chatbot/
├── backend/                    # FastAPI + Python
│   ├── pyproject.toml         # uv dependencies
│   ├── .python-version        # Python 3.11+
│   ├── src/
│   │   ├── api/               # FastAPI routes
│   │   ├── agents/            # OpenAI Agent definitions
│   │   ├── services/          # Qdrant, Neon, OpenAI clients
│   │   ├── models/            # Pydantic models
│   │   └── main.py            # FastAPI app entrypoint
│   └── tests/
│
└── frontend/                   # ChatKit + React
    ├── package.json
    ├── tsconfig.json
    ├── src/
    │   ├── components/        # ChatKit widgets
    │   ├── hooks/             # useOpenAiGlobal, etc.
    │   └── index.tsx          # Main component
    └── dist/                  # Build output (component.js)
```

**Rationale**:
- Clear separation of concerns
- Independent dependency management (uv for Python, npm for JS)
- Follows constitution guidelines for multi-language projects
- Simplifies CI/CD and deployment

### 2. Dynamic Chunk Retrieval

**Decision**: Adjust retrieval count based on query complexity AND user expertise level

**Implementation**:
```python
def calculate_chunk_count(query: str, expertise_level: str) -> int:
    """
    Calculate optimal number of chunks to retrieve.
    
    - Simple query + Advanced user = 3 chunks
    - Complex query + Beginner user = 10 chunks
    - Default = 5 chunks
    """
    # Analyze query complexity (keywords, question type, length)
    complexity_score = analyze_query_complexity(query)
    
    # Expertise multiplier
    expertise_weights = {
        'beginner': 1.5,      # More context for beginners
        'intermediate': 1.0,   # Baseline
        'advanced': 0.7        # Less context for advanced users
    }
    
    base_chunks = 5
    adjusted = int(base_chunks * complexity_score * expertise_weights[expertise_level])
    
    # Clamp between 3 and 10
    return max(3, min(10, adjusted))
```

### 3. Session Management

**Decision**: 15-minute server-side in-memory session timeout with 10-turn history limit

**Implementation**:
```python
from datetime import datetime, timedelta
from collections import deque

class ConversationSession:
    def __init__(self, session_id: str, max_turns: int = 10):
        self.session_id = session_id
        self.messages = deque(maxlen=max_turns)
        self.last_activity = datetime.now()
        self.timeout = timedelta(minutes=15)
    
    def is_expired(self) -> bool:
        return datetime.now() > self.last_activity + self.timeout
    
    def add_message(self, role: str, content: str):
        self.messages.append({"role": role, "content": content})
        self.last_activity = datetime.now()

# Global session store (use Redis in production)
sessions: Dict[str, ConversationSession] = {}

# Cleanup task (runs every 5 minutes)
async def cleanup_expired_sessions():
    while True:
        await asyncio.sleep(300)  # 5 minutes
        expired = [sid for sid, sess in sessions.items() if sess.is_expired()]
        for sid in expired:
            del sessions[sid]
```

### 4. Authentication Flow

**Decision**: Require Better Auth authentication before chatbot access

**Flow**:
1. User visits book page
2. Chatbot icon visible to all users
3. Unauthenticated user clicks → redirect to login modal (Better Auth)
4. After successful auth → session established → chatbot accessible
5. Authenticated user → chatbot loads with expertise level from DB

---

## Implementation Risks & Mitigations

### Risk 1: OpenAI Agents SDK Python Availability

**Risk**: OpenAI Agents SDK for Python may not be released yet (as of Dec 2024)

**Mitigation**:
- **Fallback option**: Use OpenAI Chat Completions API directly with custom agent orchestration
- **Alternative**: Use the Node.js OpenAI Agents SDK and bridge to Python backend via REST API
- **Monitoring**: Check OpenAI Python SDK releases weekly

### Risk 2: Rate Limiting

**Risk**: OpenAI API and Qdrant Cloud have rate limits

**Mitigation**:
- Implement exponential backoff with retry logic
- Display user-friendly error messages
- Queue requests during high traffic
- Monitor rate limit headers

### Risk 3: Embedding Cost

**Risk**: OpenAI embedding API costs accumulate with book size

**Mitigation**:
- Use `text-embedding-3-small` (lower cost than ada-002)
- One-time indexing during build (not per-request)
- Cache embeddings to avoid re-computation
- Estimated cost: ~$0.002 per 1K tokens (full book = ~$0.50)

---

## Next Steps

1. ✅ Research complete
2. → Design data models and API contracts
3. → Generate quickstart guide
4. → Create implementation tasks

---

## References

- FastAPI: https://fastapi.tiangolo.com/
- uv: https://github.com/astral-sh/uv
- OpenAI Agents SDK: https://openai.github.io/openai-agents-python/
- OpenAI ChatKit: https://developers.openai.com/apps-sdk/build/chatgpt-ui
- Qdrant: https://qdrant.tech/documentation/
- Neon Serverless: https://github.com/neondatabase/serverless
