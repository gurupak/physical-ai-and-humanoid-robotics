# Implementation Tasks: RAG Chatbot for Interactive Book Learning

**Feature**: 008-rag-chatbot  
**Branch**: `008-rag-chatbot`  
**Generated**: 2025-12-11

## Overview

This document breaks down the RAG chatbot implementation into actionable tasks organized by user story. Each user story phase delivers an independently testable increment of functionality.

**Total Tasks**: 68  
**Parallelization Opportunities**: 28 tasks marked [P]  
**User Stories**: 5 (3 P1, 2 P2, 1 P3)

---

## Dependencies & Execution Strategy

### User Story Completion Order

```
Phase 1: Setup (blocking for all)
   ↓
Phase 2: Foundational Infrastructure (blocking for all user stories)
   ↓
   ├→ US1 (P1): General Book Q&A [INDEPENDENT] ← MVP
   ├→ US3 (P1): Expertise Level [DEPENDS ON: US1] 
   ├→ US2 (P2): Selected Text [DEPENDS ON: US1]
   ├→ US4 (P2): UI Integration [DEPENDS ON: US1, US3]
   └→ US5 (P3): Conversation History [DEPENDS ON: US1]
   ↓
Phase 8: Polish & Integration
```

**MVP Scope**: US1 only (General Book Q&A) - delivers core value  
**Increment 2**: + US3 (Expertise Level)  
**Increment 3**: + US2 + US4 (Selected Text + UI)  
**Increment 4**: + US5 (Conversation History)

### Parallel Execution Opportunities

**Phase 2 Foundational** (after setup complete):
- T010-T013 can run in parallel (different services)
- T014-T016 can run in parallel (different scripts)

**Phase 3 US1** (after foundational complete):
- T020, T021, T022 can run in parallel (different agents)
- T024, T025 can run in parallel (different tools)

**Phase 7 Frontend** (independent of backend):
- T057-T062 can run in parallel (different components)

---

## Phase 1: Setup & Project Initialization

**Goal**: Initialize project structure, dependencies, and development environment.

**Tasks**:

- [ ] T001 Create chatbot root directory structure: `chatbot/backend/` and `chatbot/frontend/`
- [ ] T002 Initialize backend Python project with uv: `cd chatbot/backend && uv init && uv venv`
- [ ] T003 Create `chatbot/backend/pyproject.toml` with dependencies: fastapi>=0.109.0, uvicorn[standard]>=0.27.0, pydantic>=2.6.0, python-dotenv>=1.0.0, openai>=1.10.0, qdrant-client>=1.7.0, asyncpg>=0.29.0, python-multipart>=0.0.6, pytest>=8.0.0, pytest-asyncio>=0.23.0, ruff>=0.2.0, mypy>=1.8.0
- [ ] T004 Create `chatbot/backend/.python-version` with Python 3.11
- [ ] T005 [P] Initialize frontend project: `cd chatbot/frontend && npm init -y`
- [ ] T006 [P] Create `chatbot/frontend/package.json` with dependencies: react@^18.2.0, react-dom@^18.2.0, @types/react@^18.2.0, @types/react-dom@^18.2.0, esbuild@^0.20.0, typescript@^5.3.0
- [ ] T007 [P] Create `chatbot/frontend/tsconfig.json` with strict TypeScript config
- [ ] T008 Create `.env.example` with required environment variables: OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY, DATABASE_URL
- [ ] T009 Create backend directory structure: `src/api/`, `src/agents/`, `src/services/`, `src/models/`, `src/middleware/`, `src/utils/`, `scripts/`, `tests/`

---

## Phase 2: Foundational Infrastructure

**Goal**: Set up shared services and infrastructure required by all user stories.

**Independent Test**: Health check endpoint returns status for all services (database, vector DB, AI service).

**Tasks**:

- [ ] T010 [P] Create configuration module in `chatbot/backend/src/config.py` loading environment variables with validation
- [ ] T011 [P] Implement Neon Postgres client in `chatbot/backend/src/services/neon.py` with async connection pool using asyncpg
- [ ] T012 [P] Implement Qdrant client in `chatbot/backend/src/services/qdrant.py` with async search operations
- [ ] T013 [P] Implement OpenAI client wrapper in `chatbot/backend/src/services/openai_client.py` for embeddings and completions
- [ ] T014 [P] Create Qdrant collection setup script in `chatbot/backend/scripts/setup_qdrant.py` (collection: book_content, vectors: 1536 dims, cosine distance)
- [ ] T015 [P] Implement text chunking utility in `chatbot/backend/src/utils/chunking.py` (chunk by section/subsection, 512 tokens max, 50 token overlap)
- [ ] T016 [P] Implement embedding generation utility in `chatbot/backend/src/utils/indexing.py` using text-embedding-3-small
- [ ] T017 Create book indexing script in `chatbot/backend/scripts/index_book.py` to parse MDX files from docs/ and upload to Qdrant with metadata
- [ ] T018 Implement FastAPI app initialization in `chatbot/backend/src/main.py` with CORS middleware for GitHub Pages origin
- [ ] T019 Implement health check endpoint GET /api/health in `chatbot/backend/src/api/health.py` checking database, vector_db, ai_service status

---

## Phase 3: US1 (P1) - General Book Question Answering

**User Story**: A reader asks questions about book content and receives accurate, expertise-tailored answers with citations.

**Independent Test**: Ask "What is a Vision-Language-Action model?" and verify response includes answer grounded in book content with chapter/section citations.

**Acceptance Criteria**:
- ✅ Authenticated reader can ask questions via chat interface
- ✅ System retrieves relevant chunks from Qdrant (dynamic count based on complexity)
- ✅ System generates accurate answer grounded in retrieved content
- ✅ Response includes citations (chapter, section, line range)
- ✅ Response time < 5 seconds for typical queries

**Tasks**:

- [ ] T020 [P] [US1] Implement Triage Agent in `chatbot/backend/src/agents/triage.py` with handoff logic to specialist agents
- [ ] T021 [P] [US1] Implement Book Q&A Agent in `chatbot/backend/src/agents/book_qa.py` with instructions for general questions
- [ ] T022 [P] [US1] Implement Selected Text Agent in `chatbot/backend/src/agents/selected_text.py` with instructions for text-specific questions
- [ ] T023 [US1] Implement dynamic chunk count calculation in `chatbot/backend/src/utils/retrieval.py` based on query complexity analysis (range: 3-10 chunks)
- [ ] T024 [P] [US1] Implement search_book_content tool in `chatbot/backend/src/agents/book_qa.py` calling Qdrant semantic search
- [ ] T025 [P] [US1] Implement analyze_selection tool in `chatbot/backend/src/agents/selected_text.py` prioritizing selected text context
- [ ] T026 [US1] Implement citation formatting utility in `chatbot/backend/src/utils/retrieval.py` extracting chapter, section, line_range from metadata
- [ ] T027 [US1] Create Pydantic models in `chatbot/backend/src/models/chat.py`: ChatRequest, ChatResponse, Citation
- [ ] T028 [US1] Create Pydantic models in `chatbot/backend/src/models/session.py`: ConversationSession with 15-min timeout and 10-turn deque
- [ ] T029 [US1] Implement session store (in-memory dict) in `chatbot/backend/src/services/session_manager.py` with get/create/cleanup methods
- [ ] T030 [US1] Implement session cleanup background task in `chatbot/backend/src/services/session_manager.py` running every 5 minutes
- [ ] T031 [US1] Implement authentication middleware in `chatbot/backend/src/middleware/auth.py` validating Better Auth session tokens
- [ ] T032 [US1] Implement rate limiting middleware in `chatbot/backend/src/middleware/rate_limit.py` (10 requests/min per user)
- [ ] T033 [US1] Implement POST /api/chat endpoint in `chatbot/backend/src/api/chat.py` with authentication, request validation, agent orchestration, and error handling
- [ ] T034 [US1] Add error handling for OpenAI rate limits in `chatbot/backend/src/api/chat.py` with retry-after header
- [ ] T035 [US1] Add error handling for Qdrant failures in `chatbot/backend/src/api/chat.py` with service-specific error messages
- [ ] T036 [US1] Test US1: Ask general question "What is a Vision-Language-Action model?" and verify answer with citations

---

## Phase 4: US3 (P1) - Expertise Level Customization

**User Story**: A reader sets their expertise level (beginner, intermediate, advanced) and all responses are tailored to their understanding.

**Independent Test**: Set expertise to "beginner", ask a complex question, and verify response uses simple language. Change to "advanced" and verify response includes technical depth.

**Acceptance Criteria**:
- ✅ Reader can select expertise level on first use
- ✅ Expertise level persists in Neon Postgres users.expertiseLevel column
- ✅ Reader can change expertise level anytime
- ✅ Response complexity adapts based on expertise level
- ✅ Dynamic chunk retrieval adjusts for expertise (beginners get 1.5x chunks, advanced get 0.7x)

**Tasks**:

- [ ] T037 [P] [US3] Create Pydantic models in `chatbot/backend/src/models/user.py`: ExpertiseUpdateRequest, ExpertiseUpdateResponse, ExpertiseLevel enum
- [ ] T038 [US3] Implement GET user expertise from Neon in `chatbot/backend/src/services/neon.py` querying users.expertiseLevel by user_id
- [ ] T039 [US3] Implement PUT /api/user/expertise endpoint in `chatbot/backend/src/api/user.py` updating users.expertiseLevel in Neon
- [ ] T040 [US3] Update dynamic chunk count calculation in `chatbot/backend/src/utils/retrieval.py` to apply expertise weights (beginner: 1.5x, intermediate: 1.0x, advanced: 0.7x)
- [ ] T041 [US3] Update agent instructions in `chatbot/backend/src/agents/book_qa.py` to include expertise level in system prompt with tailored response guidelines
- [ ] T042 [US3] Update ChatRequest model in `chatbot/backend/src/models/chat.py` to fetch and include user expertise level from Neon
- [ ] T043 [US3] Update POST /api/chat endpoint in `chatbot/backend/src/api/chat.py` to retrieve user expertise level and pass to agents
- [ ] T044 [US3] Test US3: Set expertise to beginner, ask question, verify simple language; change to advanced, verify technical depth

---

## Phase 5: US2 (P2) - Text Selection-Based Questions

**User Story**: A reader selects text, asks a question, and receives an answer specifically grounded in the selected passage.

**Independent Test**: Select a paragraph about digital twins, ask "Explain this in simpler terms", and verify response references the selected text explicitly.

**Acceptance Criteria**:
- ✅ Reader can submit questions with selected text context
- ✅ Selected text is prioritized as primary context
- ✅ Response explains the selected passage specifically
- ✅ Works for code snippets with step-by-step breakdown
- ✅ Handles multi-paragraph selections

**Tasks**:

- [ ] T045 [US2] Update ChatRequest model in `chatbot/backend/src/models/chat.py` to accept optional selected_text field (max 10,000 chars)
- [ ] T046 [US2] Update Triage Agent in `chatbot/backend/src/agents/triage.py` to detect selected_text and route to Selected Text Agent
- [ ] T047 [US2] Update analyze_selection tool in `chatbot/backend/src/agents/selected_text.py` to embed selected text and search for related chunks
- [ ] T048 [US2] Update agent instructions in `chatbot/backend/src/agents/selected_text.py` to prioritize selected text in response generation
- [ ] T049 [US2] Test US2: Select text about digital twins, ask "Explain in simpler terms", verify response references selected text

---

## Phase 6: US4 (P2) - Chatbot UI Integration in Book

**User Story**: An authenticated reader sees a chatbot icon on every book page, clicks it, and uses the chat interface without leaving the page.

**Independent Test**: Navigate to any Docusaurus page, click chatbot icon, send a message, and verify response appears in the chat widget. Test on mobile and desktop.

**Acceptance Criteria**:
- ✅ Chatbot icon visible on all book pages
- ✅ Clicking icon opens chat overlay/sidebar
- ✅ Chat interface is responsive (mobile + desktop)
- ✅ Unauthenticated users see login prompt when clicking icon
- ✅ Chat doesn't disrupt reading (overlay can be dismissed)

**Tasks**:

- [ ] T050 [US4] Update ChatRequest model to require session_id from Better Auth
- [ ] T051 [US4] Implement authentication check in POST /api/chat returning 401 for unauthenticated requests
- [ ] T052 [US4] Test backend authentication: Call POST /api/chat without token and verify 401 response
- [ ] T053 [US4] Test backend with valid token: Call POST /api/chat with Better Auth token and verify 200 response

---

## Phase 7: Frontend Implementation (ChatKit Widget)

**Goal**: Build React ChatKit widget embedded in Docusaurus for chat interface.

**Independent Test**: Load widget in browser, type message, verify message appears in UI (without backend integration initially).

**Tasks**:

- [ ] T054 Create frontend directory structure: `src/components/`, `src/hooks/`, `src/types/`, `src/utils/`
- [ ] T055 Create `chatbot/frontend/src/types/openai.d.ts` declaring window.openai types (OpenAiGlobals, API methods)
- [ ] T056 Implement useOpenAiGlobal hook in `chatbot/frontend/src/hooks/useOpenAiGlobal.ts` using useSyncExternalStore for window.openai
- [ ] T057 [P] [US4] Implement ChatWidget component in `chatbot/frontend/src/components/ChatWidget.tsx` with message list and input box
- [ ] T058 [P] [US4] Implement MessageList component in `chatbot/frontend/src/components/MessageList.tsx` displaying user and assistant messages
- [ ] T059 [P] [US4] Implement InputBox component in `chatbot/frontend/src/components/InputBox.tsx` with send button and loading state
- [ ] T060 [P] [US3] [US4] Implement SettingsPanel component in `chatbot/frontend/src/components/SettingsPanel.tsx` for expertise level selection
- [ ] T061 [P] [US4] Implement CitationLinks component in `chatbot/frontend/src/components/CitationLinks.tsx` rendering clickable chapter/section links
- [ ] T062 [P] [US4] Implement ErrorBoundary component in `chatbot/frontend/src/components/ErrorBoundary.tsx` catching React errors
- [ ] T063 [US4] Implement useWidgetState hook in `chatbot/frontend/src/hooks/useWidgetState.ts` persisting state via window.openai.setWidgetState
- [ ] T064 [US4] Implement useChatSession hook in `chatbot/frontend/src/hooks/useChatSession.ts` managing message history and API calls
- [ ] T065 [US4] Create API client in `chatbot/frontend/src/utils/api.ts` for POST /api/chat with fetch and credentials: 'include'
- [ ] T066 [US4] Create main component entry in `chatbot/frontend/src/index.tsx` mounting ChatWidget and handling window.openai globals
- [ ] T067 [US4] Configure esbuild in `chatbot/frontend/package.json` script "build": bundle src/index.tsx to dist/component.js in ESM format
- [ ] T068 [US4] Build frontend: Run `npm run build` and verify `dist/component.js` created
- [ ] T069 [US4] Embed ChatKit widget in Docusaurus: Create React component in `src/components/ChatbotWidget/index.tsx` importing dist/component.js with BrowserOnly wrapper
- [ ] T070 [US4] Test frontend UI: Load Docusaurus page, click chatbot icon, send message, verify it appears in message list
- [ ] T071 [US4] Test expertise level selector: Open settings, change expertise, verify UI reflects change
- [ ] T072 [US4] Test responsive design: Test chat widget on mobile viewport (< 768px) and desktop, verify usability

---

## Phase 8: US5 (P3) - Persistent Conversation History

**User Story**: A reader asks multiple related questions and the chatbot remembers previous exchanges for coherent multi-turn conversations.

**Independent Test**: Ask "What are neural networks?", then ask "Can you give an example?" without mentioning neural networks, and verify response provides a neural network example.

**Acceptance Criteria**:
- ✅ Chatbot maintains context for up to 10 conversation turns
- ✅ Follow-up questions reference previous context
- ✅ Closing and reopening widget preserves history (same session)
- ✅ Reader can clear conversation to start fresh
- ✅ Session expires after 15 minutes of inactivity

**Tasks**:

- [ ] T073 [US5] Update ConversationSession model in `chatbot/backend/src/models/session.py` to include add_message and get_context methods
- [ ] T074 [US5] Update POST /api/chat endpoint in `chatbot/backend/src/api/chat.py` to store user query and assistant response in session
- [ ] T075 [US5] Update agent orchestration in `chatbot/backend/src/api/chat.py` to pass conversation history from session.get_context() to agents
- [ ] T076 [US5] Test US5: Ask "What are neural networks?", then ask "Give an example?" and verify response references neural networks

---

## Phase 9: Integration & End-to-End Testing

**Goal**: Integrate all components and validate complete user flows.

**Tasks**:

- [ ] T077 Run book indexing script: `uv run python chatbot/backend/scripts/index_book.py --docs-dir docs/` and verify chunks in Qdrant
- [ ] T078 Start backend server: `cd chatbot/backend && uv run uvicorn src.main:app --reload` and verify health check
- [ ] T079 Test complete US1 flow: Open Docusaurus, login, click chatbot, ask "What is a VLA model?", verify answer with citations
- [ ] T080 Test complete US3 flow: Change expertise level via settings, ask question, verify response complexity matches level
- [ ] T081 Test complete US2 flow: Select text passage, ask question, verify response explains selected text
- [ ] T082 Test complete US5 flow: Ask multi-turn conversation (5+ turns), verify context maintained
- [ ] T083 Test error handling: Disconnect Qdrant, send query, verify error message displayed to user
- [ ] T084 Test rate limiting: Send 15 requests rapidly, verify rate limit error after 10th request
- [ ] T085 Test session timeout: Wait 16 minutes, send query, verify new session created
- [ ] T086 Test unauthenticated access: Click chatbot without login, verify redirect to Better Auth login

---

## Phase 10: Polish & Cross-Cutting Concerns

**Goal**: Add finishing touches, documentation, and deployment configuration.

**Tasks**:

- [ ] T087 [P] Create `chatbot/backend/README.md` with setup instructions, environment variables, and running guide
- [ ] T088 [P] Create `chatbot/frontend/README.md` with build instructions and component architecture
- [ ] T089 [P] Add type checking to frontend: Run `npm run type-check` and fix TypeScript errors
- [ ] T090 [P] Add linting to backend: Run `uv run ruff check src/` and fix issues
- [ ] T091 [P] Add code formatting to backend: Run `uv run ruff format src/`
- [ ] T092 Configure Vercel deployment for backend in `vercel.json` with environment variables
- [ ] T093 Configure GitHub Actions for frontend build and deployment to GitHub Pages
- [ ] T094 Create deployment guide in `chatbot/DEPLOYMENT.md` with Vercel setup, environment variables, and troubleshooting
- [ ] T095 Update root README.md with chatbot feature section and links to documentation
- [ ] T096 Final smoke test: Deploy to production, test all user stories end-to-end

---

## Task Summary

### By Phase

| Phase | User Story | Task Count | Parallelizable |
|-------|------------|------------|----------------|
| 1 | Setup | 9 | 3 |
| 2 | Foundational | 10 | 6 |
| 3 | US1 (P1) General Q&A | 17 | 6 |
| 4 | US3 (P1) Expertise Level | 8 | 1 |
| 5 | US2 (P2) Selected Text | 5 | 0 |
| 6 | US4 (P2) UI Integration Backend | 4 | 0 |
| 7 | Frontend (US4 + US3) | 19 | 6 |
| 8 | US5 (P3) Conversation History | 4 | 0 |
| 9 | Integration & E2E | 10 | 0 |
| 10 | Polish | 10 | 4 |
| **TOTAL** | | **96** | **26** |

### By User Story

| User Story | Priority | Task Count | Dependencies |
|------------|----------|------------|--------------|
| Setup | - | 9 | None |
| Foundational | - | 10 | Setup |
| US1: General Q&A | P1 | 17 | Foundational |
| US3: Expertise Level | P1 | 8 | US1 |
| US2: Selected Text | P2 | 5 | US1 |
| US4: UI Integration | P2 | 23 (4 backend + 19 frontend) | US1, US3 |
| US5: Conversation History | P3 | 4 | US1 |
| Integration | - | 10 | All user stories |
| Polish | - | 10 | Integration |

---

## MVP Definition

**Minimum Viable Product (US1 only)**:
- Tasks T001-T036 (43 tasks)
- Delivers: Authenticated readers can ask questions and receive expertise-tailored answers with citations
- Excludes: Expertise customization UI, selected text, conversation history, polished frontend

**Time Estimate**: ~2-3 weeks for single developer (MVP), 4-6 weeks for full feature

---

## Notes

- **No test tasks included**: Spec does not explicitly request TDD approach. Testing is done via acceptance scenarios in each phase.
- **Parallelization**: 26 tasks marked [P] can run in parallel with other tasks in the same phase.
- **Independent testing**: Each user story phase includes "Independent Test" criteria for validation without dependencies.
- **Incremental delivery**: Each phase delivers a working increment that can be demoed/validated.
- **Environment setup**: Requires `.env` with OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY, DATABASE_URL before T010.
- **Better Auth assumption**: Assumes Better Auth is already functional in `api-server/` with users.expertiseLevel column configured.
