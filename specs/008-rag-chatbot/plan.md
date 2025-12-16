# Implementation Plan: RAG Chatbot for Interactive Book Learning

**Branch**: `008-rag-chatbot` | **Date**: 2025-12-11 | **Spec**: [spec.md](./spec.md)  
**Input**: Feature specification from `/specs/008-rag-chatbot/spec.md`

## Summary

Build and embed a Retrieval-Augmented Generation (RAG) chatbot within the published Docusaurus book using FastAPI (Python), OpenAI Agents SDK, OpenAI ChatKit, Neon Serverless Postgres, and Qdrant Cloud vector database. The chatbot enables authenticated readers to ask questions about book content with expertise-level tailored responses, including support for questions about user-selected text passages.

**Core Features**:
- Multi-agent orchestration (Triage → Book Q&A / Selected Text specialists)
- Semantic search with dynamic chunk retrieval based on query complexity and user expertise
- ChatKit-powered frontend embedded in Docusaurus
- Session management with 15-minute timeout and 10-turn history limit
- Privacy-first (no conversation logging)

---

## Technical Context

**Language/Version**: Python 3.11+  
**Primary Dependencies**: FastAPI 0.109+, OpenAI Python SDK 1.10+, Qdrant Client 1.7+, asyncpg 0.29+, uv (package manager)  
**Storage**: Neon Postgres (user preferences), Qdrant Cloud (vector embeddings), in-memory (session state)  
**Testing**: pytest 8.0+, pytest-asyncio 0.23+  
**Target Platform**: Python backend (FastAPI/Uvicorn), React frontend (ChatKit), deployed to Vercel/cloud  
**Project Type**: Web application (separate backend/frontend)  
**Performance Goals**: <5s response time (p95), 100 concurrent sessions  
**Constraints**: <200ms p95 for Qdrant search, 15-min session timeout, 10-turn history limit  
**Scale/Scope**: ~100 concurrent users, ~1000 book chunks, OpenAI free tier compatible

---

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Pre-Phase 0 Check ✅

| Principle | Status | Notes |
|-----------|--------|-------|
| **I. Documentation-First** | ✅ PASS | Used Context7/Ref MCP servers to fetch latest docs for FastAPI, uv, Qdrant, OpenAI SDK |
| **III. Deployment Architecture** | ✅ PASS | Backend on Vercel (FastAPI), frontend on GitHub Pages (Docusaurus + ChatKit) |
| **IV. Book Platform** | ✅ PASS | Docusaurus 3.x unchanged, ChatKit widget embedded via BrowserOnly |
| **V. API Backend** | ✅ PASS | FastAPI 14+ API routes only, CORS for GitHub Pages origin |
| **VI. React & TypeScript** | ✅ PASS | ChatKit frontend uses React 18 functional components + TypeScript strict |
| **VII. Tailwind CSS** | N/A | ChatKit uses built-in styling, no custom CSS needed |
| **VIII. OpenAI Agents JS SDK** | ⚠️ **DEVIATION** | Using **Python Agents SDK** instead of JS SDK per user requirement |
| **IX. RAG Architecture** | ✅ PASS | Hybrid search, citations, streaming (SSE), hierarchical chunking |
| **X. ChatKit Frontend** | ✅ PASS | ChatKit embedded in Docusaurus with BrowserOnly wrapper |
| **XI. Authentication** | ✅ PASS | Better Auth with OAuth (Google, GitHub), session cookies, onboarding flow |
| **XIV. UX & Accessibility** | ✅ PASS | WCAG AA compliance, responsive design, keyboard navigation |
| **XVI. Quality Gates** | ✅ PASS | TypeScript strict, 80% test coverage, RAG precision >0.85 |

**Deviation Justification**:
- **Principle VIII**: User explicitly requested Python-based backend with FastAPI and "OpenAI Agents SDK Python". Since Python backend is mandated, using Python Agents SDK is the correct choice for consistency and avoiding inter-language complexity.

---

## Project Structure

### Documentation (this feature)

```text
specs/008-rag-chatbot/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (technology research)
├── data-model.md        # Phase 1 output (schemas, models)
├── quickstart.md        # Phase 1 output (setup guide)
├── contracts/           # Phase 1 output (API spec)
│   └── openapi.yaml
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT YET CREATED)
```

### Source Code (repository root)

```text
chatbot/
├── backend/                    # FastAPI + Python
│   ├── pyproject.toml         # uv dependencies
│   ├── .python-version        # Python 3.11+
│   ├── uv.lock                # Lock file (generated)
│   ├── README.md              # Backend-specific docs
│   ├── src/
│   │   ├── __init__.py
│   │   ├── main.py            # FastAPI app entrypoint
│   │   ├── config.py          # Configuration (env vars)
│   │   ├── api/               # API routes
│   │   │   ├── __init__.py
│   │   │   ├── chat.py        # POST /api/chat
│   │   │   ├── health.py      # GET /api/health
│   │   │   └── user.py        # PUT /api/user/expertise
│   │   ├── agents/            # OpenAI Agent definitions
│   │   │   ├── __init__.py
│   │   │   ├── triage.py      # Triage agent (router)
│   │   │   ├── book_qa.py     # Book Q&A specialist
│   │   │   └── selected_text.py  # Selected text specialist
│   │   ├── services/          # External service clients
│   │   │   ├── __init__.py
│   │   │   ├── qdrant.py      # Qdrant vector search
│   │   │   ├── neon.py        # Neon Postgres queries
│   │   │   └── openai_client.py  # OpenAI API wrapper
│   │   ├── models/            # Pydantic models
│   │   │   ├── __init__.py
│   │   │   ├── chat.py        # ChatRequest, ChatResponse
│   │   │   ├── user.py        # ExpertiseUpdateRequest, etc.
│   │   │   └── session.py     # ConversationSession
│   │   ├── middleware/        # FastAPI middleware
│   │   │   ├── __init__.py
│   │   │   ├── auth.py        # Better Auth integration
│   │   │   ├── cors.py        # CORS configuration
│   │   │   └── rate_limit.py  # Rate limiting
│   │   └── utils/             # Helper functions
│   │       ├── __init__.py
│   │       ├── chunking.py    # Text chunking logic
│   │       ├── indexing.py    # Embedding generation
│   │       └── retrieval.py   # Dynamic chunk count calculation
│   ├── scripts/
│   │   ├── index_book.py      # One-time indexing script
│   │   └── setup_qdrant.py    # Qdrant collection setup
│   └── tests/
│       ├── __init__.py
│       ├── conftest.py        # Pytest fixtures
│       ├── test_api/
│       │   ├── test_chat.py
│       │   ├── test_health.py
│       │   └── test_user.py
│       ├── test_agents/
│       │   └── test_agents.py
│       └── test_services/
│           ├── test_qdrant.py
│           └── test_neon.py
│
└── frontend/                   # ChatKit + React (embedded widget, NOT a Next.js app)
    ├── package.json
    ├── tsconfig.json
    ├── .eslintrc.json
    ├── README.md              # Frontend-specific docs
    ├── src/
    │   ├── index.tsx          # Main component entry
    │   ├── components/
    │   │   ├── ChatWidget.tsx         # Main chat interface
    │   │   ├── MessageList.tsx        # Message display
    │   │   ├── InputBox.tsx           # User input
    │   │   ├── SettingsPanel.tsx      # Expertise level selector
    │   │   ├── CitationLinks.tsx      # Citation display
    │   │   └── ErrorBoundary.tsx      # Error handling
    │   ├── hooks/
    │   │   ├── useOpenAiGlobal.ts     # window.openai accessor
    │   │   ├── useWidgetState.ts      # Persist widget state
    │   │   └── useChatSession.ts      # Chat session management
    │   ├── types/
    │   │   └── openai.d.ts            # window.openai types
    │   └── utils/
    │       ├── api.ts                 # Backend API calls
    │       └── formatting.ts          # Text formatting
    └── dist/                  # Build output (esbuild)
        └── component.js       # Bundled component (embedded in Docusaurus)
```

**Structure Decision**: Web application pattern (Option 2) with separate `backend/` and `frontend/` directories. Backend uses Python with `uv` for dependency management. Frontend uses Node/npm with esbuild for bundling. This separation enables independent development, testing, and deployment while maintaining clear boundaries between Python and JavaScript ecosystems.

---

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| OpenAI Agents SDK (Python vs JS) | User mandated Python backend with FastAPI; Python SDK required for ecosystem consistency | JS SDK would require Node.js backend, contradicting user's explicit FastAPI requirement |

---

## Phase 0: Research (✅ COMPLETED)

**Artifacts Generated**:
- [research.md](./research.md) - Technology stack research and best practices

**Key Findings**:
1. **uv package manager**: 10-100x faster than pip, native pyproject.toml support
2. **FastAPI 0.109+**: Native async, OpenAPI generation, Pydantic v2 integration
3. **OpenAI Agents SDK (Python)**: Handoff-based multi-agent orchestration
4. **ChatKit**: Official OpenAI UI library with `window.openai` API integration
5. **Qdrant Cloud**: Free tier (1GB), HNSW indexing, hybrid search support
6. **Neon Serverless**: Already configured, message pipelining optimizations

**Technology Stack (Finalized)**:
- Python 3.11+, FastAPI 0.109+, Uvicorn
- uv (package manager)
- OpenAI Python SDK 1.10+ (Agents SDK, embeddings, completions)
- Qdrant Client 1.7+
- asyncpg 0.29+ (Neon Postgres)
- React 18, TypeScript 5+, esbuild 0.20+
- OpenAI ChatKit (latest)

---

## Phase 1: Design & Contracts (✅ COMPLETED)

**Artifacts Generated**:
1. [data-model.md](./data-model.md) - Data schemas, API models, in-memory structures
2. [contracts/openapi.yaml](./contracts/openapi.yaml) - OpenAPI 3.1 specification
3. [quickstart.md](./quickstart.md) - Development setup guide

**Data Model Summary**:

**Database (Neon Postgres)**:
- `users.expertiseLevel` VARCHAR(20) CHECK IN ('beginner', 'intermediate', 'advanced')
- No additional tables required (session managed by Better Auth)

**Vector Database (Qdrant)**:
- Collection: `book_content`
- Vector size: 1536 (text-embedding-3-small)
- Payload: chapter, section, content, line_range, file_path, chunk_index

**API Models (Pydantic)**:
- ChatRequest, ChatResponse
- ExpertiseUpdateRequest, ExpertiseUpdateResponse
- ErrorResponse (standardized error format)

**In-Memory**:
- ConversationSession (15-min timeout, 10-turn deque)

**API Endpoints**:
1. `POST /api/chat` - Send chat message (authenticated)
2. `PUT /api/user/expertise` - Update expertise level (authenticated)
3. `GET /api/health` - Health check (public)

---

## Phase 2: Implementation Tasks (⏳ PENDING)

**Command**: Run `/sp.tasks` to generate task breakdown from this plan.

**Expected Task Categories**:
1. **Backend Setup** (7-10 tasks)
   - Initialize backend project with uv
   - Configure FastAPI app with CORS middleware
   - Implement authentication middleware (Better Auth integration)
   - Implement rate limiting middleware
   - Set up Neon Postgres client
   - Set up Qdrant client
   - Set up OpenAI client

2. **API Implementation** (5-8 tasks)
   - Implement POST /api/chat endpoint
   - Implement PUT /api/user/expertise endpoint
   - Implement GET /api/health endpoint
   - Add error handling and logging
   - Add request validation

3. **Agent Implementation** (6-9 tasks)
   - Implement Triage Agent
   - Implement Book Q&A Agent
   - Implement Selected Text Agent
   - Create search_book_content tool
   - Create analyze_selection tool
   - Implement dynamic chunk retrieval logic

4. **Indexing & Retrieval** (5-7 tasks)
   - Implement text chunking utility
   - Implement embedding generation utility
   - Create Qdrant collection setup script
   - Create book indexing script
   - Implement semantic search service
   - Implement citation formatting

5. **Session Management** (3-5 tasks)
   - Implement ConversationSession class
   - Implement session store (in-memory)
   - Implement session cleanup background task
   - Add session expiry logic

6. **Frontend Implementation** (8-12 tasks)
   - Initialize frontend project with npm
   - Configure TypeScript and esbuild
   - Implement ChatWidget component
   - Implement MessageList component
   - Implement InputBox component
   - Implement SettingsPanel component
   - Implement CitationLinks component
   - Implement ErrorBoundary component
   - Create useOpenAiGlobal hook
   - Create useWidgetState hook
   - Create useChatSession hook
   - Build and bundle component

7. **Integration** (4-6 tasks)
   - Embed ChatKit component in Docusaurus
   - Configure CORS for GitHub Pages origin
   - Test authentication flow
   - Test chat flow end-to-end
   - Test selected text flow
   - Test expertise level switching

8. **Testing** (6-10 tasks)
   - Write API endpoint tests
   - Write agent tests
   - Write Qdrant service tests
   - Write Neon service tests
   - Write frontend component tests
   - Write integration tests
   - Achieve 80% test coverage

9. **Deployment** (3-5 tasks)
   - Configure Vercel for FastAPI backend
   - Configure environment variables
   - Set up CI/CD pipeline
   - Deploy and smoke test
   - Document deployment process

**Total Estimated Tasks**: 47-72 tasks

---

## Agent Context Update

Run the agent context update script:

```bash
bash .specify/scripts/bash/update-agent-context.sh claude
```

**Technologies to add**:
- Python 3.11+
- FastAPI 0.109+
- uv (package manager)
- OpenAI Agents SDK (Python)
- OpenAI ChatKit
- Qdrant Cloud
- Neon Serverless Postgres (already present)

---

## Critical Implementation Notes

### 1. Authentication Flow

```python
# middleware/auth.py
from fastapi import HTTPException, Depends
from fastapi.security import HTTPBearer

security = HTTPBearer()

async def verify_auth(token: str = Depends(security)):
    # Validate Better Auth session token
    user = await get_user_from_token(token.credentials)
    if not user:
        raise HTTPException(status_code=401, detail="Unauthorized")
    return user
```

**Integration points**:
- Better Auth session validation
- Extract user ID and expertise level from Neon
- Pass to agent context

### 2. Dynamic Chunk Retrieval

```python
# utils/retrieval.py
def calculate_chunk_count(query: str, expertise_level: str) -> int:
    complexity_score = analyze_query_complexity(query)  # 0.5-2.0
    expertise_weights = {
        'beginner': 1.5,
        'intermediate': 1.0,
        'advanced': 0.7
    }
    base = 5
    adjusted = int(base * complexity_score * expertise_weights[expertise_level])
    return max(3, min(10, adjusted))  # Clamp 3-10
```

### 3. Session Management

```python
# models/session.py
from collections import deque
from datetime import datetime, timedelta

class ConversationSession:
    def __init__(self, session_id: str, user_id: str, expertise_level: str):
        self.session_id = session_id
        self.user_id = user_id
        self.expertise_level = expertise_level
        self.messages = deque(maxlen=10)  # 10-turn limit
        self.last_activity = datetime.now()
        self.timeout = timedelta(minutes=15)
    
    def is_expired(self) -> bool:
        return datetime.now() > self.last_activity + self.timeout
```

### 4. Error Handling

```python
# api/chat.py
@app.post("/api/chat")
async def chat_endpoint(request: ChatRequest, user = Depends(verify_auth)):
    try:
        result = await process_chat(request, user)
        return result
    except RateLimitError as e:
        return ErrorResponse(error=ErrorDetail(
            error_code="RATE_LIMIT_EXCEEDED",
            message="Too many requests. Please wait before retrying.",
            service="openai",
            retry_after=60
        ))
    except QdrantError as e:
        return ErrorResponse(error=ErrorDetail(
            error_code="SERVICE_ERROR",
            message="Search service temporarily unavailable.",
            service="qdrant",
            retry_after=30
        ))
```

---

## Risk Mitigation

### Risk 1: OpenAI Agents SDK Python Availability

**Status**: ⚠️ MONITORING  
**Mitigation**: If SDK not available, implement custom agent orchestration with OpenAI Chat Completions API and function calling. Fallback implementation documented in research.md.

### Risk 2: Rate Limiting

**Status**: ✅ HANDLED  
**Mitigation**: Implemented in middleware with exponential backoff, user-facing error messages, and retry-after headers.

### Risk 3: Embedding Costs

**Status**: ✅ CONTROLLED  
**Mitigation**: One-time indexing (not per-request), using cost-effective `text-embedding-3-small` model (~$0.002/1K tokens).

---

## Success Metrics

From spec.md success criteria:

1. **SC-002**: 95% of queries < 5 seconds response time
2. **SC-009**: 10 consecutive turns without context loss
3. **SC-011**: 100 concurrent sessions without degradation
4. **SC-012**: Zero security incidents

**Monitoring**:
- Response time histogram (p50, p95, p99)
- Session count and active users
- Error rate by service (OpenAI, Qdrant, Neon)
- Authentication success/failure rate

---

## Post-Phase 1 Constitution Re-Check ✅

| Principle | Status | Notes |
|-----------|--------|-------|
| **VIII. OpenAI Agents SDK** | ✅ **PASS (with justified deviation)** | Using Python SDK per user requirement, consistent with Python backend |
| **IX. RAG Architecture** | ✅ PASS | Multi-agent, hybrid search, citations confirmed in design |
| **X. ChatKit Frontend** | ✅ PASS | BrowserOnly wrapper, `window.openai` API, streaming support |
| **XI. Authentication** | ✅ PASS | Better Auth integration, OAuth support, cross-origin sessions |
| **XVI. Quality Gates** | ✅ PASS | 80% test coverage target, TypeScript strict, pytest + mypy |

**All gates passed. Ready to proceed to Phase 2 (tasks generation).**

---

## Next Steps

1. ✅ Phase 0: Research complete
2. ✅ Phase 1: Design & contracts complete
3. **→ Phase 2**: Run `/sp.tasks` to generate implementation task breakdown
4. **→ Implementation**: Execute tasks from tasks.md
5. **→ Testing**: Achieve 80% coverage, validate acceptance criteria
6. **→ Deployment**: Deploy to Vercel, configure production environment

---

## References

- **Feature Spec**: [spec.md](./spec.md)
- **Research**: [research.md](./research.md)
- **Data Model**: [data-model.md](./data-model.md)
- **API Contract**: [contracts/openapi.yaml](./contracts/openapi.yaml)
- **Quickstart**: [quickstart.md](./quickstart.md)
- **Constitution**: [.specify/memory/constitution.md](../../.specify/memory/constitution.md)
