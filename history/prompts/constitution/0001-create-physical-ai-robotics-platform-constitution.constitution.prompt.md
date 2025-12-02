---
id: 0001
title: Create Physical AI Robotics Platform Constitution
stage: constitution
date: 2025-11-28
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: developer
command: /sp.constitution
labels: ["constitution", "governance", "principles", "robotics", "ai", "nextjs", "react", "fastapi"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - None
---

## Prompt

```text
/constitution Create governing principles for a high-tech Physical AI & Humanoid Robotics textbook with embedded RAG chatbot.

**Documentation-First Development:**
- Always fetch latest library documentation via MCP servers before implementation
- Use Context7 MCP for library-specific docs: FastAPI, OpenAI Agents SDK, ChatKit, Docusaurus, Next.js, Tailwind CSS, React
- No implementation proceeds without consulting current API references

**Research Tools for Book Content:**
- Use Tavily MCP for: crawling official docs (ros.org, nvidia.com, gazebosim.org), site mapping, current robotics news
- Use Exa MCP for: code examples, research papers (arxiv), semantic concept search
- Cross-reference multiple authoritative sources before writing any chapter section
- Prioritize official documentation over blog posts and tutorials

**Book Content Quality Standards:**
- All code examples must be tested and runnable against pinned library versions
- Version lock all dependencies: specify exact versions (ROS 2 Humble, Isaac Sim 2023.1.1, Python 3.11)
- Hardware-related code must include safety warnings and preconditions
- Every chapter must state prerequisites explicitly at the start
- Complex architectures require diagrams (Mermaid or images), not just prose
- Include "Common Errors" sections for tricky implementations
- Provide both "Quick Start" and "Deep Dive" paths per topic

**Book Structure (Docusaurus):**
- Content in MDX format covering Physical AI & Humanoid Robotics
- Flexible chapter structure: minimum 3 chapters, maximum 10 chapters
- Core topics: ROS 2, Gazebo/Unity, NVIDIA Isaac, Vision-Language-Action, hardware/lab setup
- Clear prerequisite chains between chapters with visual dependency graph
- Difficulty indicators per section: Beginner, Intermediate, Advanced
- Estimated reading time displayed for each chapter
- GitHub Pages deployment via GitHub Actions mandatory
- RTL support for Urdu translation (i18n)

**Frontend Tech Stack (Latest Versions):**
- Next.js 14+ with App Router (NOT Pages Router)
- React 18+ with Server Components as default
- Tailwind CSS v4.1 with CSS-first @theme configuration
- TypeScript strict mode enabled

**Next.js App Router Requirements:**
- Use file-system based routing: app/page.tsx, app/layout.tsx
- Default to React Server Components (RSC) - only use 'use client' when necessary
- Use Server Actions for form handling and mutations (no separate API routes for forms)
- Implement streaming with Suspense boundaries for loading states
- Use generateMetadata() for dynamic SEO metadata per page
- Use middleware.ts for authentication redirects and route protection
- Use Route Handlers (app/api/route.ts) only for external API integrations
- Leverage enhanced fetch() with caching: cache, next.revalidate, next.tags
- Use revalidatePath() and revalidateTag() for on-demand cache invalidation
- Use generateStaticParams() for static route generation where applicable

**React Best Practices:**
- Functional components with hooks exclusively (no class components)
- useState for local component state
- useEffect only for external system synchronization (NOT for derived state)
- useContext for cross-component data sharing (theme, auth, user profile)
- useMemo and useCallback for expensive computations and stable references
- useTransition for non-blocking state updates (search, filters)
- useDeferredValue for deferring expensive UI updates
- React.memo() for preventing unnecessary re-renders of pure components
- Custom hooks for reusable stateful logic (useBookProgress, useChapterPersonalization)
- Suspense boundaries for code-splitting with React.lazy()

**Tailwind CSS v4.1 Standards:**
- Use CSS-first configuration with @theme directive (no tailwind.config.js)
- Define custom design tokens in CSS: --color-*, --font-*, --spacing-*
- Use @utility directive for custom utility classes
- Use @custom-variant for custom conditional styles
- Use @layer base/components for organized custom styles
- Mobile-first responsive design with breakpoint prefixes (sm:, md:, lg:, xl:)
- Dark mode via prefers-color-scheme (default) or manual toggle with @custom-variant
- Use arbitrary values [value] sparingly, prefer theme tokens
- Leverage --alpha() function for opacity modifications
- Use --spacing() function for consistent spacing calculations

**Frontend Reading Experience:**
- Typography optimized for technical content: 18px base font, 1.6 line height, max 75ch line length
- Syntax-highlighted code blocks with copy button, line numbers, and language label
- Dark/light mode toggle using Tailwind dark: variant, persisted in localStorage
- Adjustable font size (small/medium/large) for accessibility
- Expandable code examples: show simplified version, reveal full on click
- Back-to-top button on long pages
- Persistent sidebar showing chapter list and current position

**Frontend Navigation & Discovery:**
- Breadcrumb navigation using Next.js usePathname() hook
- Full-text search powered by Docusaurus search or Algolia
- "Related Topics" suggestions at end of each section
- "Next Chapter" and "Previous Chapter" navigation using Next.js Link component
- Anchor links for all headings (shareable deep links)
- Table of contents sidebar for current page (auto-generated from headings)

**Frontend Progress & Retention (Authenticated Users):**
- Reading progress saved per chapter using Server Actions
- Visual progress indicators on chapter list (checkmarks, progress bars)
- "Continue Reading" button on homepage showing last position
- Bookmarking capability using useOptimistic for instant feedback
- Chapter completion tracking with visual dashboard
- State managed via React Context for global user state

**Responsive Design Requirements:**
- Mobile-first CSS approach using Tailwind breakpoints
- Breakpoints: default (mobile), sm:640px, md:768px, lg:1024px, xl:1280px, 2xl:1536px
- Code blocks horizontally scrollable on mobile using overflow-x-auto
- Collapsible sidebar on tablet/mobile using React state and Tailwind responsive classes
- Touch targets minimum 44x44px using Tailwind sizing utilities (min-h-11 min-w-11)
- Images responsive using Next.js Image component with sizes prop
- Tables scrollable horizontally on small screens using overflow-x-auto

**Performance Requirements (Core Web Vitals):**
- Largest Contentful Paint (LCP): under 2.5 seconds
- First Input Delay (FID): under 100 milliseconds  
- Cumulative Layout Shift (CLS): under 0.1
- Time to Interactive: under 3 seconds
- Use Next.js Image component for automatic image optimization
- Use next/font for zero-layout-shift font loading
- Use React.lazy() and Suspense for code-splitting heavy components
- Use Next.js streaming and partial prerendering where applicable
- Leverage Server Components to reduce client JavaScript bundle

**Accessibility (WCAG AA Compliance):**
- All images have descriptive alt text via Next.js Image alt prop
- Color contrast ratio minimum 4.5:1 using Tailwind color palette
- Full keyboard navigation for all interactive elements
- Focus indicators using Tailwind focus: and focus-visible: variants
- Proper heading hierarchy (h1 > h2 > h3, no skips)
- ARIA labels for custom interactive components
- No information conveyed by color alone
- RTL layout fully functional for Urdu content using Tailwind RTL plugin

**SEO & Discoverability:**
- Use Next.js generateMetadata() for dynamic page metadata
- Unique meta descriptions for each chapter
- Open Graph and Twitter Card meta tags via metadata export
- JSON-LD structured data (TechArticle schema) via script tags
- Auto-generated sitemap using Next.js sitemap.ts
- Canonical URLs using Next.js metadata alternates
- Descriptive URLs using Next.js dynamic routes

**RAG Chatbot Backend (FastAPI + OpenAI Agents SDK):**
- Hybrid search: dense vectors (Qdrant) + BM25 keywords
- Hierarchical chunking: 512 tokens prose, atomic code blocks preserved whole
- SSE streaming for real-time response delivery
- Session persistence 90 days via SQLAlchemy + Neon Postgres
- Selected-text context injection for focused Q&A
- Guardrails: reject questions completely unrelated to robotics/AI
- Citation requirement: every answer must reference source chapter/section

**Chatbot Frontend (OpenAI ChatKit + React):**
- ChatKit embedded using 'use client' component wrapper
- Streaming responses with useTransition for non-blocking UI
- Message history managed via React Context + Server Actions
- Clear "New Conversation" button with useOptimistic feedback
- Selected text feature: floating button on text selection using window.getSelection()
- Graceful error handling using React Error Boundaries
- Suggested follow-up questions after each response
- Citation display linking to chapter sections using Next.js Link

**Content Personalization (Authenticated Users):**
- Onboarding questionnaire captures: Python level, ROS experience, hardware access, learning goals
- User context stored in React Context, hydrated from server via Server Components
- Personalize button triggers Server Action to regenerate chapter content
- Chatbot instructions dynamically include user background from context
- Recommend chapters based on stated learning goals

**Authentication (Better-Auth):**
- OAuth providers: Google, GitHub minimum
- Auth state managed via React Context with Server Component hydration
- Protected routes via Next.js middleware.ts
- Post-auth redirect to onboarding if questionnaire incomplete
- User profiles stored in Neon Postgres with preferences
- Session cookies set with httpOnly, secure, sameSite flags

**Internationalization:**
- Default locale: English
- Secondary locale: Urdu with RTL support
- Use Next.js internationalization routing
- Translation button triggers locale switch via Next.js navigation
- Technical terms maintain English with Urdu explanation
- Code comments remain in English
- UI strings via translation files
- Urdu translations require technical review before publication

**Quality Gates:**
- Test-first: 80% minimum backend coverage
- RAG retrieval precision > 0.85 on test query set
- WCAG AA accessibility audit pass required
- Lighthouse performance score > 90
- No chapter ships without complete documentation
- All code examples must pass automated tests in CI
- Cross-browser testing: Chrome, Firefox, Safari, Edge
- Mobile testing: iOS Safari, Android Chrome
- TypeScript strict mode with no any types

**Tech Stack Constraints:**
- Backend: Python 3.11+ with FastAPI, SQLModel, OpenAI Agents SDK
- Frontend: Next.js 14+, React 18+, TypeScript 5+, Tailwind CSS v4.1
- Database: Qdrant Cloud free tier for vectors, Neon Serverless Postgres for user data
- No premature abstractions - use framework features directly
- Prefer Server Components over Client Components when possible
- Prefer Server Actions over API routes for mutations
- Use Tailwind utilities directly, avoid @apply except for third-party overrides
- No custom CSS frameworks - Tailwind only
```

## Response snapshot

Created comprehensive constitution (v1.0.0) for Physical AI & Humanoid Robotics textbook platform with 10 core principles:

1. **Documentation-First Development** - MCP server integration for authoritative docs (Context7, Tavily, Exa)
2. **Research & Content Quality** - Tested code examples, version locking, safety warnings, prerequisite chains
3. **Next.js App Router Architecture** - RSC-first, Server Actions, streaming, proper routing patterns
4. **React & TypeScript Standards** - Functional hooks, strict TypeScript, performance optimizations
5. **Tailwind CSS v4.1 Standards** - CSS-first config, mobile-first responsive, theme tokens
6. **User Experience & Accessibility** - WCAG AA compliance, Core Web Vitals, RTL support, reading optimization
7. **RAG Chatbot Architecture** - Hybrid search, citations, streaming, session persistence
8. **Performance & Quality Gates** - 80% test coverage, accessibility audits, cross-browser testing
9. **Authentication & Security** - Better-Auth OAuth, personalization, progress tracking
10. **Internationalization & RTL** - English/Urdu support, SEO optimization

Constitution includes governance framework with version semantics (MAJOR.MINOR.PATCH), amendment procedures, complexity justification requirements, and compliance review checkpoints.

Sync impact report embedded in constitution file. All template files validated for compatibility.

## Outcome

- ✅ Impact: Established comprehensive governance framework for educational robotics platform
- 🧪 Tests: No tests required (governance document)
- 📁 Files: Created .specify/memory/constitution.md (v1.0.0)
- 🔁 Next prompts: Feature specification or planning with /sp.specify or /sp.plan
- 🧠 Reflection: Constitution provides clear quality gates and architectural constraints for complex dual-stack (Next.js + FastAPI) educational platform

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Template validation PASS, placeholder resolution PASS
- Prompt variant (if applicable): constitution-comprehensive-v1
- Next experiment (smallest change to try): Monitor constitution effectiveness during first feature specification
