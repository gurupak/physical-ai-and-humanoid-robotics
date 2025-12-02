---
id: 0003
title: Update constitution v3.0.0 split platform
stage: constitution
date: 2025-12-02
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: system
command: /sp.constitution
labels: ["constitution", "architecture", "split-platform", "docusaurus", "github-pages", "vercel"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

```text
some more updates in it as follow "
/constitution Create governing principles for a high-tech Physical AI & Humanoid Robotics textbook with embedded RAG chatbot.

**Documentation-First Development:**

- Always fetch latest library documentation via MCP servers before implementation

- Use Context7 MCP for library docs: Docusaurus, React, Tailwind CSS, OpenAI Agents JS SDK, ChatKit, Next.js

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

**Deployment Architecture (Split Platform):**

- Book Frontend: Docusaurus 3.x deployed to GitHub Pages (static)

- API Backend: Next.js API routes deployed to Vercel (serverless)

- ChatKit widget embedded in Docusaurus, calls Vercel API

- CORS configured to allow GitHub Pages domain to call Vercel API

- Environment variables: GitHub secrets for Pages, Vercel dashboard for API

**Book Platform (Docusaurus 3.x + GitHub Pages):**

- Content in MDX format covering Physical AI & Humanoid Robotics

- Flexible chapter structure: minimum 3 chapters, maximum 10 chapters

- Core topics: ROS 2, Gazebo/Unity, NVIDIA Isaac, Vision-Language-Action, hardware setup

- Use docs-only mode (no blog)

- TypeScript configuration

- Tailwind CSS for custom component styling

- Built-in search via Docusaurus search plugin or Algolia

- i18n configured for English (default) and Urdu (RTL)

- GitHub Actions workflow for automatic deployment on push to main

- Clear prerequisite chains between chapters

- Difficulty indicators: Beginner, Intermediate, Advanced

- Estimated reading time per chapter

**API Backend (Next.js on Vercel):**

- Next.js 14+ with App Router (API routes only, no pages)

- TypeScript strict mode enabled

- OpenAI Agents JS SDK (@openai/agents) for agent orchestration

- Zod v3 for parameter validation

- @qdrant/js-client-rest for vector operations

- @neondatabase/serverless for Postgres

- Better-Auth for authentication

- SSE streaming for chat responses

- CORS middleware allowing requests from GitHub Pages domain

**OpenAI Agents JS SDK Usage:**

- Create agents with: new Agent({ name, instructions, tools, handoffs })

- Define tools with: tool({ name, description, parameters: z.object(...), execute })

- Use handoffs for multi-agent routing (triage agent → specialist agents)

- Use OpenAIConversationsSession for conversation memory

- Use built-in webSearchTool() if web search needed

**RAG Chatbot Architecture:**

- Book Q&A Agent: answers questions from textbook content

- Selected Text Agent: answers from user-highlighted text only

- Triage Agent: routes between general Q&A and selected text modes

- Hybrid search: dense vectors (Qdrant) + keyword matching

- Hierarchical chunking: 512 tokens prose, atomic code blocks preserved

- Streaming responses via Vercel API with SSE

- Session persistence in Neon Postgres

- Citation requirement: every answer references source chapter/section

**Chatbot Frontend (ChatKit in Docusaurus):**

- ChatKit embedded as custom Docusaurus component

- Wrapped in BrowserOnly to avoid SSR issues

- Streaming responses with typing indicator

- Message history for authenticated users

- Selected text feature: floating button on text selection

- Error handling with retry mechanism

- Citation display linking to chapter sections

**Authentication (Better-Auth via Vercel API):**

- OAuth providers: Google, GitHub minimum

- Auth endpoints on Vercel: /api/auth/*

- Onboarding questionnaire: Python level, ROS experience, hardware access, learning goals

- User profiles stored in Neon Postgres

- Session tokens stored in httpOnly cookies

- Docusaurus components check auth state via API call

**Content Personalization:**

- Onboarding questionnaire stored in Neon Postgres

- Dynamic agent instructions based on user profile

- Personalize button at chapter start calls Vercel API

- Chatbot considers user background when answering

**Internationalization (Docusaurus i18n):**

- Default locale: English

- Secondary locale: Urdu with RTL support (direction: 'rtl' in localeConfigs)

- Translation files in i18n/ur/docusaurus-plugin-content-docs/

- Translation button switches locale via Docusaurus routing

- Technical terms maintain English with Urdu explanation

**Frontend Reading Experience:**

- Typography optimized: 18px base font, 1.6 line height, max 75ch

- Syntax-highlighted code blocks with copy button and line numbers

- Dark/light mode via Docusaurus color mode

- Expandable code examples

- Back-to-top button

- Persistent sidebar showing chapter list

**Performance Requirements:**

- Largest Contentful Paint (LCP): under 2.5 seconds

- First Input Delay (FID): under 100 milliseconds

- Cumulative Layout Shift (CLS): under 0.1

- Docusaurus static generation for fast loads

- Lazy load images below fold

- Code-split heavy components

**Accessibility (WCAG AA):**

- All images have descriptive alt text

- Color contrast ratio minimum 4.5:1

- Full keyboard navigation

- Focus indicators visible

- Proper heading hierarchy

- Screen reader compatible

- RTL layout functional for Urdu

**Quality Gates:**

- TypeScript strict mode with no any types

- ESLint + Prettier enforced

- 80% test coverage for API routes

- RAG retrieval precision > 0.85

- Lighthouse performance score > 90

- WCAG AA accessibility audit pass

- All code examples tested in CI

**Tech Stack Summary:**

- Book: Docusaurus 3.x, React 18, TypeScript, Tailwind CSS, MDX

- API: Next.js 14+ (API routes only), TypeScript, OpenAI Agents JS SDK

- Auth: Better-Auth

- Database: Neon Serverless Postgres

- Vectors: Qdrant Cloud

- Validation: Zod v3

- Book Deployment: GitHub Pages via GitHub Actions

- API Deployment: Vercel"
```

## Response snapshot

Successfully updated constitution from v2.0.0 to v3.0.0. This is a MAJOR version bump due to complete architectural transformation:

**Key Changes:**
- Architecture: Unified Next.js full-stack → Split platform (Docusaurus on GitHub Pages + Next.js API on Vercel)
- Book Frontend: Reintroduced Docusaurus 3.x for static documentation site
- API Backend: Next.js API routes only (no pages), with CORS middleware for cross-origin requests
- Deployment: Split between GitHub Pages (book) and Vercel (API)
- ChatKit Integration: Embedded in Docusaurus using BrowserOnly wrapper
- i18n: Docusaurus native i18n with English/Urdu RTL support
- Environment Variables: Split between GitHub secrets (build-time) and Vercel dashboard (runtime)

**New Principles Added:**
- III. Deployment Architecture (Split Platform)
- IV. Book Platform (Docusaurus 3.x + GitHub Pages)
- V. API Backend (Next.js on Vercel)

**Modified Principles:**
- I. Documentation-First Development - Added Docusaurus to Context7 MCP list
- VIII. Chatbot Frontend - Updated for Docusaurus embedding
- X. Internationalization - Docusaurus i18n configuration
- XI. Frontend Reading Experience - Docusaurus-specific features

**Constitution Version:** 3.0.0 | **Ratified:** 2025-12-02 | **Last Amended:** 2025-12-02

## Outcome

- ✅ Impact: Complete constitutional update with MAJOR version bump (2.0.0 → 3.0.0) covering split platform architecture
- 🧪 Tests: No automated tests required for constitutional amendments
- 📁 Files: Updated `.specify/memory/constitution.md` with 884 lines covering 16 core principles
- 🔁 Next prompts: Consider running `/sp.plan` or `/sp.specify` to create features aligned with new split platform architecture
- 🧠 Reflection: Split platform architecture optimizes costs (GitHub Pages free + Vercel free tier) while maintaining clear separation between static content and dynamic API features. CORS configuration is critical for cross-origin communication.

## Evaluation notes (flywheel)

- Failure modes observed: None - constitutional update completed successfully
- Graders run and results (PASS/FAIL): Manual review PASS - all placeholders filled, sync impact report updated, version semantics followed
- Prompt variant (if applicable): Standard constitution update flow via /sp.constitution command
- Next experiment (smallest change to try): Monitor template propagation to plan/spec/tasks templates as flagged in sync impact report
