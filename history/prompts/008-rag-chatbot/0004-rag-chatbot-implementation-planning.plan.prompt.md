---
id: "0004"
title: "RAG Chatbot Implementation Planning"
stage: plan
date: 2025-12-11
surface: RAG Chatbot Feature Implementation Plan
model: claude-sonnet-4-5-20250929
feature: 008-rag-chatbot
branch: 008-rag-chatbot
user: developer
command: /sp.plan
labels: [plan, rag-chatbot, fastapi, python, openai-agents, chatkit, qdrant, architecture]
links:
  spec: specs/008-rag-chatbot/spec.md
  ticket: ""
  adr: ""
  pr: ""
files:
  - path: specs/008-rag-chatbot/plan.md
    status: created
    lines_changed: 650
  - path: specs/008-rag-chatbot/research.md
    status: created
    lines_changed: 450
  - path: specs/008-rag-chatbot/data-model.md
    status: created
    lines_changed: 380
  - path: specs/008-rag-chatbot/quickstart.md
    status: created
    lines_changed: 420
  - path: specs/008-rag-chatbot/contracts/openapi.yaml
    status: created
    lines_changed: 450
  - path: CLAUDE.md
    status: modified
    lines_changed: 10
tests: []
---

## Prompt

User requested implementation planning for RAG chatbot feature with specific technology requirements:

**User Input**: "we will be use FASTAPI, OpenAI Agents SDK Python, OpenAI chatkit, neon postgres, qdrant cloud vector DB. Use latest info on all from context7 and ref mcp servers. The chatbot will work after authentication is done, which is already implemented through better auth. We need to push one time the book data into qdrant vector DB. Need to enter qdrant connection, let me know the env variable in .env to add. Write clean code. use uv python library to initialize the backend server. get latest on uv from context7 mcp server. Make proper chatbot folder and under it create backend and frontend folder."

**Context**: Feature spec completed with 10 clarifications. All ambiguities resolved regarding authentication requirements, chunk retrieval strategy, session management, and error handling.

## Response snapshot

Completed comprehensive implementation planning workflow:

**Phase 0: Research & Technology Selection**
- Researched FastAPI best practices (async patterns, dependency injection, middleware)
- Researched OpenAI Agents SDK Python (handoff-based orchestration, tool integration)
- Researched uv package manager (10-100x faster than pip, native pyproject.toml support)
- Researched Qdrant Cloud (vector search, hybrid search, Python client)
- Researched OpenAI ChatKit (`window.openai` API, React integration, streaming)
- Researched Neon Serverless (asyncpg for Python, connection pooling)

**Phase 1: Design & Contracts**
- Defined data models (Pydantic schemas, database schemas, in-memory structures)
- Defined API contracts (OpenAPI 3.1 specification with 3 endpoints)
- Created project structure (separate backend/frontend with proper organization)
- Documented dynamic chunk retrieval logic (based on query complexity + expertise level)
- Documented session management (15-min timeout, 10-turn history)

**Artifacts Created**:
1. `research.md` - Technology stack research (450 lines)
2. `data-model.md` - Data schemas and models (380 lines)
3. `contracts/openapi.yaml` - OpenAPI specification (450 lines)
4. `quickstart.md` - Developer setup guide (420 lines)
5. `plan.md` - Implementation plan (650 lines)

**Key Architectural Decisions**:
1. **Project structure**: `chatbot/backend/` (Python/FastAPI) + `chatbot/frontend/` (React/ChatKit)
2. **Multi-agent architecture**: Triage → Book Q&A / Selected Text specialists
3. **Dynamic retrieval**: 3-10 chunks based on query complexity × expertise weight
4. **Session management**: In-memory with 15-min TTL and 10-turn circular buffer
5. **Authentication**: Better Auth integration (already implemented)
6. **Indexing**: One-time build script using text-embedding-3-small

**Environment Variables Specified**:
```env
OPENAI_API_KEY=sk-...
QDRANT_URL=https://your-cluster-name.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key
DATABASE_URL=postgresql://... (already configured)
```

**Constitution Check**: ✅ All principles passed with one justified deviation (Python Agents SDK instead of JS SDK per user requirement).

## Outcome

- ✅ Impact: Complete implementation plan with 5 design artifacts covering technology research, data modeling, API contracts, project structure, and developer onboarding. Ready for task generation phase.
- 🧪 Tests: No test code (planning phase). Test coverage targets specified: 80% backend, integration tests for all endpoints.
- 📁 Files: 5 new files (research.md, data-model.md, quickstart.md, openapi.yaml, plan.md), 1 modified (CLAUDE.md with updated tech stack)
- 🔁 Next prompts: Run `/sp.tasks` to generate detailed implementation task breakdown (estimated 47-72 tasks across 9 categories)
- 🧠 Reflection: Comprehensive planning completed with research-backed technology decisions. All user requirements addressed including environment variable specification, project structure, and clean code practices. Dynamic chunk retrieval and session management strategies align with performance constraints.

## Evaluation notes (flywheel)

- Failure modes observed: None. Planning workflow completed successfully with all artifacts generated.
- Graders run and results (PASS/FAIL): N/A (planning phase, no code)
- Prompt variant (if applicable): Standard /sp.plan workflow with user-specified technology stack
- Next experiment (smallest change to try): Proceed to task generation (/sp.tasks) to break down implementation into actionable units
