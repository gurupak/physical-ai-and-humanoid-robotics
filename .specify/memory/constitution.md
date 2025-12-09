# Physical AI & Humanoid Robotics Textbook Platform Constitution

**Version**: 3.0.0 | **Ratified**: 2025-12-02 | **Last Amended**: 2025-12-02

<!--
SYNC IMPACT: v2.0.0 → v3.0.0 (MAJOR) - Split platform architecture (Docusaurus on GitHub Pages + Next.js API on Vercel)
Modified: I, III, IV, V, VIII, X, XI | Added: III, IV, V | Removed: Unified Next.js references
Templates: plan-template.md, spec-template.md, tasks-template.md require updates
-->

## Core Principles

### I. Documentation-First Development

**Rule**: Consult current API references from authoritative sources before implementation.

- Fetch latest library docs via MCP servers (Context7, Tavily, Exa)
- Priority: Context7 for Docusaurus, React, Tailwind, OpenAI Agents JS SDK, ChatKit, Next.js
- Cross-reference multiple authoritative sources for educational content
- Check Docusaurus 3.x and OpenAI Agents JS SDK docs for API patterns

**Rationale**: Prevents outdated patterns, ensures API accuracy, reduces technical debt. Educational content requires maximum accuracy.

---

### II. Research & Content Quality

**Rule**: All educational content must be tested, versioned, and validated.

Requirements:
- Code examples tested against pinned versions (ROS 2 Humble, Isaac Sim 2023.1.1, Python 3.11)
- Safety warnings for hardware-related code
- Explicit prerequisites at chapter start
- Diagrams for complex architectures (Mermaid/images)
- "Common Errors" sections with troubleshooting
- "Quick Start" (15-min) and "Deep Dive" paths per topic

**Rationale**: Educational integrity demands accuracy. Version pinning prevents breakage. Safety warnings prevent damage/injury.

---

### III. Deployment Architecture (Split Platform)

**Rule**: Book frontend on GitHub Pages (static); API backend on Vercel (serverless).

**Book Frontend (Docusaurus → GitHub Pages)**:
- Docusaurus 3.x static site via GitHub Actions
- Custom domain optional
- Environment variables via GitHub secrets (build-time only)
- No server-side logic

**API Backend (Next.js → Vercel)**:
- Next.js 14+ App Router (API routes only, NO pages)
- Handles: chatbot, auth, user progress, RAG
- Environment variables via Vercel dashboard (OPENAI_API_KEY, QDRANT_*, DATABASE_URL, auth keys)
- CORS middleware for GitHub Pages origin (see reference docs for implementation)

**Communication**: ChatKit widget → Vercel API via fetch() with credentials

**Rationale**: Optimizes costs (GitHub Pages free, Vercel free tier). Static frontend = fast loads. Serverless scales automatically.

---

### IV. Book Platform (Docusaurus 3.x + GitHub Pages)

**Rule**: Docusaurus 3.x docs-only mode with TypeScript; 3-10 chapters.

**Core Setup**:
- TypeScript config (`docusaurus.config.ts`)
- Docs-only mode: `blog: false, docs: { routeBasePath: '/' }`
- MDX format (React component embedding)
- 3-10 chapters: ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA, hardware

**Features**:
- Tailwind CSS via custom plugin/PostCSS
- Dark/light mode (built-in)
- Search (local plugin or Algolia)
- i18n: English (default), Urdu (RTL)
- Chapter metadata: difficulty, readingTime, prerequisites
- GitHub Actions deployment (`peaceiris/actions-gh-pages@v3`)

**Rationale**: Battle-tested docs platform. Free hosting. MDX enables interactive components. Flexible structure.

---

### V. API Backend (Next.js on Vercel)

**Rule**: Next.js 14+ App Router, API routes only, CORS enabled.

**Configuration**:
- App Router (`app/api/**/*.ts`)
- NO pages, NO frontend rendering
- TypeScript strict mode
- CORS middleware for GitHub Pages (see `.specify/docs/api-patterns.md`)

**Routes**:
- `/api/chat`: Chatbot SSE streaming
- `/api/auth/*`: Better-Auth endpoints
- `/api/progress`: User reading progress
- `/api/personalize`: Content adaptation

**Dependencies**: @openai/agents, zod, @qdrant/js-client-rest, @neondatabase/serverless, better-auth

**Rationale**: API-only reduces complexity. Vercel native optimization. SSE enables streaming.

---

### VI. React & TypeScript Standards

**Rule**: Functional components with hooks; TypeScript strict mode, no `any`.

**React Patterns**:
- Hooks: useState, useEffect (external sync only), useContext, useMemo, useCallback
- useTransition/useDeferredValue for non-blocking updates
- React.memo() for pure components
- Custom hooks: useAuth(), useChatSession(), useReadingProgress()
- Suspense + React.lazy() for code-splitting

**TypeScript**:
- Strict mode, no `any` (use `unknown` then narrow)
- Explicit return types for exported functions
- Discriminated unions for state machines
- Zod schemas for API validation

**Rationale**: Hooks = cleaner state management. TypeScript strict catches bugs early.

---

### VII. Tailwind CSS Standards

**Rule**: Tailwind for all styling; integrate with Docusaurus.

**Use**:
- Custom design tokens in `tailwind.config.js`
- Mobile-first responsive (`sm:`, `md:`, `lg:`, `xl:`, `2xl:`)
- Dark mode via `dark:` variant (synced with Docusaurus)
- Integration via Docusaurus plugin/PostCSS

**Avoid**:
- `@apply` (except Docusaurus overrides)
- Custom CSS frameworks
- Inline styles
- Magic numbers without docs

**Rationale**: Utility-first scales with components. Mobile-first ensures accessibility.

---

### VIII. OpenAI Agents JS SDK Patterns

**Rule**: Use @openai/agents for orchestration; follow recommended patterns.

**Agent Creation**: Use `new Agent({ name, instructions, tools, handoffs, model })`

**Tools**: Define with `tool({ name, description, parameters: z.object(...), execute })`

**Handoffs**: Triage → specialist agents (no circular handoffs)

**Sessions**: `OpenAIConversationsSession` with Neon Postgres backend

See `.specify/docs/agent-patterns.md` for code examples.

**Rationale**: Battle-tested orchestration. Type safety with Zod. Session management prevents context reloading.

---

### IX. RAG Chatbot Architecture

**Rule**: Hybrid search with citations; streaming responses required.

**Multi-Agent**:
- Triage Agent → Book Q&A Agent / Selected Text Agent
- All use OpenAI Agents JS SDK
- Tools for hybrid search (Qdrant + BM25)

**RAG Pipeline**:
- Hierarchical chunking: 512 tokens, 50 overlap (code blocks preserved whole)
- Embedding: text-embedding-3-small
- Reranking: top 20 → top 5
- Citations: `chapter/section:line-range` format

**API**: SSE streaming via TransformStream, Zod validation, rate limiting (10 msg/min free tier)

**Rationale**: Grounded answers prevent hallucinations. Multi-agent enables specialization. Streaming = better UX.

---

### X. Chatbot Frontend (ChatKit in Docusaurus)

**Rule**: ChatKit embedded in Docusaurus; wrapped in BrowserOnly.

**Integration**: BrowserOnly wrapper prevents SSR issues, dynamic import of ChatKit

**Features**:
- Streaming responses
- Message history (authenticated)
- Selected text context injection
- Citation links to chapters
- Error boundaries with retry

**API Calls**: fetch() with `credentials: 'include'` for cross-origin auth

See `.specify/docs/chatkit-integration.md` for implementation.

**Rationale**: ChatKit = production-ready UI. BrowserOnly prevents SSR errors. Citations deepen learning.

---

### XI. Authentication & Content Personalization

**Rule**: Better-Auth with OAuth; secure cross-origin sessions.

**Auth**:
- OAuth: Google, GitHub (minimum)
- Vercel endpoints: `/api/auth/*`
- Session cookies: `httpOnly`, `secure`, `sameSite: 'none'`
- Expiration: 30 days inactive, 90 days absolute

**Onboarding**: Capture Python level, ROS experience, hardware access, learning goals

**Personalization**: User profile → adapted content suggestions, chatbot instructions

**Rationale**: OAuth reduces friction. Cross-origin sessions enable split platform. Personalization improves outcomes.

---

### XII. Internationalization (Docusaurus i18n)

**Rule**: English default, Urdu RTL supported; technical terms stay English.

**Config**: `defaultLocale: 'en'`, `locales: ['en', 'ur']`, `ur: { direction: 'rtl' }`

**Files**: `i18n/ur/docusaurus-plugin-content-docs/current/*.md`

**Technical Terms**: English + Urdu explanation in parentheses

**Rationale**: RTL enables Urdu speakers. English terms prevent translation ambiguity.

---

### XIII. Frontend Reading Experience

**Rule**: Typography optimized; persistent navigation; keyboard shortcuts.

**Typography**:
- 18px base, 1.6 line height, max 75ch width
- Syntax highlighting with copy button, line numbers
- Dark/light mode, adjustable font size

**Navigation**:
- Breadcrumbs, full-text search, anchor links
- Next/Previous chapter, TOC sidebar
- Keyboard shortcuts: `/` search, `n` next, `p` previous

**Rationale**: Optimal reading for technical content. Keyboard shortcuts = power user experience.

---

### XIV. User Experience & Accessibility

**Rule**: WCAG AA compliance; Core Web Vitals thresholds.

**Accessibility**:
- Descriptive alt text, 4.5:1 contrast ratio
- Keyboard navigation, focus indicators
- Proper heading hierarchy, ARIA labels
- RTL layout for Urdu

**Performance**:
- LCP < 2.5s, FID < 100ms, CLS < 0.1
- Lazy load images, code-split heavy components
- < 200KB initial JS bundle

**Responsive**: Mobile-first, touch targets 44x44px minimum

**Rationale**: Accessibility = inclusive education. Performance affects engagement.

---

### XV. Book Structure & Content Quality

**Rule**: 3-10 chapters with strict quality standards.

**Structure**:
- MDX format, 3-10 chapters
- Topics: ROS 2, Gazebo/Unity, Isaac, VLA, hardware
- Metadata: difficulty, readingTime, prerequisites

**Chapter Must Include**:
- Learning objectives (3-5)
- Prerequisites with links
- Quick Start (15-min) + Deep Dive
- Runnable code examples
- Architecture diagrams
- Common errors section
- Safety warnings (hardware)
- Further reading, exercises

**Version Pinning**: ROS 2 Humble, Isaac Sim 2023.1.1, Python 3.11, all packages pinned

**Rationale**: Flexible structure. Strict quality ensures value. Version pinning prevents breakage.

---

### XVI. Performance & Quality Gates

**Rule**: No feature ships without passing all gates.

**Must Pass**:
- TypeScript strict, zero errors
- ESLint + Prettier, zero warnings
- 80% test coverage (API routes)
- RAG precision > 0.85
- WCAG AA audit pass
- Lighthouse > 90
- All code examples tested in CI
- Cross-browser testing (Chrome, Firefox, Safari, Edge)

**CI Checks**: type-check, lint, test, test:e2e, test:rag, test:examples, Lighthouse CI

**Rationale**: High reliability for education. Broken examples damage trust. Automated gates prevent regressions.

---

## Technology Stack Constraints

**Non-Negotiable**:

**Book**: Docusaurus 3.x, React 18+, TypeScript 5+, Tailwind CSS, MDX

**API**: Node.js 18+, Next.js 14+ (API routes only), TypeScript 5+, OpenAI Agents JS SDK, ChatKit, Zod, Qdrant, Neon Postgres, Better-Auth

**Deployment**: GitHub Pages (book), Vercel (API)

**Patterns**: No premature abstractions, functional components + hooks, Tailwind utilities directly, NO custom CSS frameworks

**Rationale**: Prevents bikeshedding. Free tiers enable low-cost deployment.

---

## Development Workflow

**Execution Contract**:
1. Confirm surface and success criteria
2. List constraints, invariants, non-goals
3. Produce artifact with acceptance checks
4. Add follow-ups and risks (max 3)
5. Create PHR in `history/prompts/`
6. Suggest ADR if significant decisions made

**Minimum Acceptance**:
- Clear, testable acceptance criteria
- Explicit error paths and constraints
- Smallest viable change
- Code references where relevant

**Human as Tool**:
- Ask 2-3 clarifying questions for ambiguity
- Surface unforeseen dependencies
- Present architectural options with tradeoffs
- Summarize and confirm at checkpoints

**Default Policies**:
- Clarify and plan first
- No invented APIs; ask clarifiers
- No hardcoded secrets
- Smallest viable diff
- Cite code with references (`file:line`)
- Keep reasoning private

**Code Standards**:
- Functional patterns (pure functions, immutability)
- Result types for error handling
- Structured logging
- Comments explain "why" not "what"

---

## Governance

**Authority**: This constitution supersedes all other practices. All PRs/reviews/designs must verify compliance.

**Amendment Procedure**:
1. Propose with rationale
2. Document in ADR if significant
3. Update constitution with version bump (semantic versioning)
4. Update sync impact report
5. Propagate to templates
6. Create migration plan

**Version Semantics**:
- MAJOR: Backward incompatible changes
- MINOR: New principles/guidance
- PATCH: Clarifications, typos

**Compliance Review**:
- Check before Phase 0 research
- Re-check after Phase 1 design
- Automated linting in CI
- Manual review for architectural decisions

**Detailed References**:
- Agent patterns: `.specify/docs/agent-patterns.md`
- API patterns: `.specify/docs/api-patterns.md`
- ChatKit integration: `.specify/docs/chatkit-integration.md`
- Templates: `.specify/templates/`
