---
id: 0002
title: Constitution v2.0 Physical AI Textbook Platform
stage: constitution
date: 2025-12-02
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: system
command: /sp.constitution
labels: ["constitution", "architecture", "nextjs", "openai-agents", "rag-chatbot", "vercel", "major-version"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - None (documentation update)
---

## Prompt

Create governing principles for a high-tech Physical AI & Humanoid Robotics textbook with embedded RAG chatbot.

**Documentation-First Development:**
- Always fetch latest library documentation via MCP servers before implementation
- Use Context7 MCP for library docs: Next.js, React, Tailwind CSS, OpenAI Agents JS SDK, ChatKit, Docusaurus
- No implementation proceeds without consulting current API references

**Research Tools for Book Content:**
- Use Tavily MCP for: crawling official docs (ros.org, nvidia.com, gazebosim.org), site mapping, current news
- Use Exa MCP for: code examples, research papers (arxiv), semantic concept search
- Cross-reference multiple authoritative sources before writing any chapter section
- Prioritize official documentation over blog posts and tutorials

**Book Content Quality Standards:**
- All code examples must be tested and runnable against pinned library versions
- Version lock all dependencies (ROS 2 Humble, Isaac Sim 2023.1.1, Python 3.11 for robotics code)
- Hardware-related code must include safety warnings
- Every chapter must state prerequisites at the start
- Complex architectures require diagrams (Mermaid or images)
- Include "Common Errors" sections for tricky implementations

**Book Structure (Docusaurus on Next.js):**
- Content in MDX format covering Physical AI & Humanoid Robotics
- Flexible chapter structure: minimum 3 chapters, maximum 10 chapters
- Core topics: ROS 2, Gazebo/Unity, NVIDIA Isaac, Vision-Language-Action, hardware setup
- Clear prerequisite chains between chapters
- Difficulty indicators: Beginner, Intermediate, Advanced
- Estimated reading time per chapter
- Vercel deployment with automatic preview deployments on PR

**Deployment Architecture (Single Platform - Vercel):**
- Full-stack Next.js application deployed on Vercel
- No separate backend deployment required
- API Routes handle all backend logic
- Edge functions for low-latency responses where applicable
- Environment variables managed via Vercel dashboard
- Preview deployments for every pull request

**Frontend Tech Stack:**
- Next.js 14+ with App Router (NOT Pages Router)
- React 18+ with Server Components as default
- TypeScript strict mode enabled
- Tailwind CSS v4.1 with CSS-first @theme configuration
- OpenAI ChatKit SDK for embedded chat widget
- Zod for runtime validation

**Backend Tech Stack (Next.js API Routes):**
- OpenAI Agents JS SDK (@openai/agents) for agent orchestration
- Zod v3 for parameter validation in tools
- @qdrant/js-client-rest for vector operations
- @neondatabase/serverless for Postgres
- Better-Auth for authentication
- Server-Sent Events (SSE) via Next.js streaming responses

**OpenAI Agents JS SDK Usage:**
- Create agents with: new Agent({ name, instructions, tools, handoffs })
- Define tools with: tool({ name, description, parameters: z.object(...), execute })
- Use handoffs for multi-agent routing (triage agent → specialist agents)
- Use OpenAIConversationsSession for conversation memory
- Use MCPServerStreamableHttp for external MCP tool integration
- Use built-in webSearchTool() for web search capability if needed

**RAG Chatbot Architecture:**
- Book Q&A Agent: answers questions from textbook content
- Selected Text Agent: answers questions specifically about user-highlighted text
- Triage Agent: routes between general Q&A and selected text modes
- Hybrid search: dense vectors (Qdrant) + keyword matching
- Hierarchical chunking: 512 tokens prose, atomic code blocks preserved whole
- Streaming responses via Next.js API Routes with SSE
- Session persistence in Neon Postgres using OpenAIConversationsSession pattern
- Citation requirement: every answer references source chapter/section

**Chatbot Frontend (ChatKit + React):**
- ChatKit embedded as 'use client' component
- Streaming responses with real-time typing indicator
- Message history persisted across sessions for authenticated users
- Selected text feature: floating "Ask about this" button on text selection
- Clear "New Conversation" button
- Error handling with retry mechanism
- Citation display linking to chapter sections via Next.js Link

**Next.js App Router Requirements:**
- Use file-system based routing: app/page.tsx, app/layout.tsx
- Default to React Server Components - only use 'use client' when necessary
- Use Server Actions for form handling and mutations
- Implement streaming with Suspense boundaries for loading states
- Use generateMetadata() for dynamic SEO metadata per page
- Use middleware.ts for authentication redirects and route protection
- Use Route Handlers (app/api/route.ts) for chat API endpoints
- Leverage enhanced fetch() with caching for external data

**React Best Practices:**
- Functional components with hooks exclusively
- useState for local component state
- useEffect only for external system synchronization
- useContext for cross-component data (theme, auth, user profile)
- useMemo and useCallback for expensive computations
- useTransition for non-blocking state updates
- React.memo() for preventing unnecessary re-renders
- Custom hooks for reusable logic (useBookProgress, useChatSession)
- Suspense boundaries for code-splitting

**Tailwind CSS v4.1 Standards:**
- Use CSS-first configuration with @theme directive
- Define custom design tokens in CSS: --color-*, --font-*, --spacing-*
- Use @utility directive for custom utility classes
- Mobile-first responsive design with breakpoint prefixes
- Dark mode via prefers-color-scheme or manual toggle
- Use arbitrary values [value] sparingly, prefer theme tokens

**Authentication (Better-Auth):**
- OAuth providers: Google, GitHub minimum
- Onboarding questionnaire: Python level, ROS experience, hardware access, learning goals
- User profiles stored in Neon Postgres
- Session management via Better-Auth with Next.js middleware integration
- Protected routes via middleware.ts checks

**Content Personalization (Authenticated Users):**
- Onboarding questionnaire stored in Neon Postgres
- Dynamic agent instructions based on user profile
- Personalize button at chapter start triggers content regeneration
- Chatbot considers user background when answering

**Internationalization:**
- Default locale: English
- Secondary locale: Urdu with RTL support
- Translation button at chapter start
- Technical terms maintain English with Urdu explanation
- UI strings via translation files

**Performance Requirements:**
- Largest Contentful Paint (LCP): under 2.5 seconds
- First Input Delay (FID): under 100 milliseconds
- Cumulative Layout Shift (CLS): under 0.1
- Use Next.js Image component for optimization
- Use next/font for zero-layout-shift fonts
- Leverage Server Components to reduce client JavaScript
- Use Vercel Edge for low-latency API responses

**Accessibility (WCAG AA):**
- All images have descriptive alt text
- Color contrast ratio minimum 4.5:1
- Full keyboard navigation
- Focus indicators visible
- Proper heading hierarchy
- Screen reader compatible

**Quality Gates:**
- TypeScript strict mode with no any types
- ESLint + Prettier enforced
- 80% test coverage for API routes
- RAG retrieval precision > 0.85
- Lighthouse performance score > 90
- WCAG AA accessibility audit pass
- All code examples tested in CI

**Tech Stack Summary:**
- Runtime: Node.js 18+
- Framework: Next.js 14+ (App Router)
- Language: TypeScript 5+
- Styling: Tailwind CSS v4.1
- AI: OpenAI Agents JS SDK (@openai/agents)
- Chat UI: OpenAI ChatKit
- Auth: Better-Auth
- Database: Neon Serverless Postgres
- Vectors: Qdrant Cloud
- Validation: Zod v3
- Deployment: Vercel

## Response snapshot

Successfully updated project constitution from v1.1.0 to v2.0.0 with complete architectural transformation:

**Version Change**: 1.1.0 → 2.0.0 (MAJOR)

**Rationale**: Complete architectural shift from dual-platform deployment (FastAPI backend on Railway + Next.js frontend on Vercel) to unified full-stack Next.js application on Vercel. This represents backward-incompatible changes requiring complete rewrite of backend services.

**Key Architectural Changes**:
1. **Backend Migration**: FastAPI Python → Next.js API Routes (TypeScript)
2. **Agent Framework**: Python OpenAI SDK → OpenAI Agents JS SDK (@openai/agents)
3. **Deployment**: Railway + Vercel → Vercel only (single platform)
4. **Session Management**: SQLAlchemy → OpenAIConversationsSession with Neon Postgres
5. **Content Framework**: Docusaurus → MDX on Next.js
6. **Styling**: Tailwind CSS v3.x → v4.1 CSS-first configuration

**New Principles Added**:
- VIII. Deployment Architecture (Single Platform - Vercel)
- IX. OpenAI Agents JS SDK Patterns
- X. Book Structure & Content Quality (expanded)
- Content Personalization (elevated from sub-section)

**Modified Principles**:
- All principles updated to reflect Next.js full-stack architecture
- RAG Chatbot Architecture completely rewritten for JavaScript implementation
- Technology Stack Constraints updated with new dependencies

**Template Consistency**: All templates remain compatible as they use technology-agnostic placeholders.

## Outcome

- ✅ Impact: Constitution v2.0.0 establishes governing principles for Physical AI & Humanoid Robotics textbook platform with unified Next.js architecture, OpenAI Agents JS SDK integration, and comprehensive quality gates
- 🧪 Tests: None required (documentation only)
- 📁 Files: 1 file modified (.specify/memory/constitution.md)
- 🔁 Next prompts: Ready for feature specification with /sp.specify, implementation planning with /sp.plan
- 🧠 Reflection: Constitution provides clear architectural direction with 13 comprehensive principles covering documentation-first development, Next.js App Router patterns, OpenAI Agents JS SDK usage, RAG architecture, accessibility, performance, and quality gates. Single-platform Vercel deployment significantly reduces operational complexity.

## Evaluation notes (flywheel)

- Failure modes observed: None - constitution successfully updated with complete architectural guidance
- Graders run and results (PASS/FAIL): 
  - ✅ PASS: All placeholder tokens replaced with concrete values
  - ✅ PASS: Version incremented correctly (MAJOR bump justified)
  - ✅ PASS: Sync impact report created with comprehensive change summary
  - ✅ PASS: All principles declarative and testable
  - ✅ PASS: No conflicting guidance between principles
- Prompt variant (if applicable): Standard /sp.constitution workflow
- Next experiment (smallest change to try): Create first feature specification using /sp.specify to validate constitution principles in practice
