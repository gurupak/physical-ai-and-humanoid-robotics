# Physical AI & Humanoid Robotics Textbook Platform Constitution

<!--
SYNC IMPACT REPORT (2025-12-02)
════════════════════════════════════════════════════════════════════════════════
Version Change: 2.0.0 → 3.0.0
Rationale: MAJOR (3.0.0) - Complete architectural transformation from unified
           Next.js full-stack to split platform architecture (Docusaurus on
           GitHub Pages + Next.js API backend on Vercel); reintroduced Docusaurus
           3.x for book frontend; ChatKit embedded in Docusaurus calling Vercel
           API; CORS configuration required; environment variables split between
           GitHub secrets and Vercel dashboard; incompatible with v2.0.0 unified
           Next.js architecture.

Modified Principles:
- I. Documentation-First Development - Added Docusaurus to Context7 MCP list
- III. Deployment Architecture - Complete rewrite: unified Vercel → split platform
  (Docusaurus on GitHub Pages + Next.js API on Vercel)
- IV. Book Platform - NEW: Docusaurus 3.x specific configuration, docs-only mode,
  GitHub Actions workflow
- V. API Backend - NEW: Next.js API routes only (no pages), CORS middleware
- VIII. Chatbot Frontend - Updated: embedded in Docusaurus with BrowserOnly wrapper
- X. Internationalization - Updated: Docusaurus i18n configuration with localeConfigs
- XI. Frontend Reading Experience - Docusaurus-specific typography and navigation

Added Sections:
- III. Deployment Architecture (Split Platform: GitHub Pages + Vercel)
- IV. Book Platform (Docusaurus 3.x + GitHub Pages)
- V. API Backend (Next.js on Vercel)

Removed Sections:
- Previous unified Next.js full-stack references
- MDX content in Next.js app directory references
- Next.js dynamic routes for chapters
- Server Actions for book content (book now static on GitHub Pages)

Templates Status:
✅ plan-template.md - Requires update with split platform architecture
⚠ spec-template.md - Verify deployment section reflects split platform
⚠ tasks-template.md - Update build/deployment tasks for dual platform

Follow-up TODOs:
- Verify GitHub Actions workflow configuration in plan/tasks
- Document CORS configuration details for GitHub Pages → Vercel API calls
- Ensure Docusaurus plugin configuration matches constitution requirements
════════════════════════════════════════════════════════════════════════════════
-->

## Core Principles

### I. Documentation-First Development

**Rule**: No implementation proceeds without consulting current API references from authoritative sources.

- MUST fetch latest library documentation via MCP servers before any implementation work
- MUST use Context7 MCP for library-specific docs: Docusaurus, React, Tailwind CSS, OpenAI Agents JS SDK, ChatKit, Next.js
- MUST use Tavily MCP for: crawling official docs (ros.org, nvidia.com, gazebosim.org), site mapping, current robotics news
- MUST use Exa MCP for: code examples, research papers (arxiv.org), semantic concept search
- MUST cross-reference multiple authoritative sources before writing any chapter section
- MUST prioritize official documentation over blog posts and tutorials
- For OpenAI Agents JS SDK: always check @openai/agents package documentation for Agent, tool, handoffs patterns
- For Docusaurus: always check Docusaurus 3.x documentation for plugin configuration, theming, and i18n

**Rationale**: Prevents outdated patterns, ensures correct API usage, reduces technical debt from deprecated features. Educational content requires maximum accuracy. MCP-first approach guarantees fresh documentation. Split platform architecture requires precise understanding of both Docusaurus and Next.js APIs.

---

### II. Research & Content Quality

**Rule**: All educational content must be tested, versioned, and validated against real implementations.

Book content MUST include:
- All code examples tested and runnable against pinned library versions
- Exact version specifications for all dependencies (ROS 2 Humble, Isaac Sim 2023.1.1, Python 3.11 for robotics examples)
- Safety warnings and preconditions for hardware-related code (robot movement, electrical connections)
- Explicit prerequisites at chapter start (software versions, hardware requirements, background knowledge)
- Diagrams (Mermaid or images) for complex architectures - never rely on prose alone for system diagrams
- "Common Errors" sections for tricky implementations with troubleshooting steps
- Both "Quick Start" (15-minute path) and "Deep Dive" (comprehensive) paths per topic

**Rationale**: Educational integrity demands accuracy. Readers following tutorials must achieve working results. Versioning prevents "works on my machine" failures. Safety warnings prevent hardware damage and physical injury.

---

### III. Deployment Architecture (Split Platform)

**Rule**: Book frontend on GitHub Pages (static); API backend on Vercel (serverless); clear separation of concerns.

Deployment architecture MUST implement:

**Book Frontend (Docusaurus → GitHub Pages)**:
- Docusaurus 3.x static site deployed to GitHub Pages
- GitHub Actions workflow for automatic deployment on push to main branch
- Static HTML/CSS/JS bundle served via GitHub Pages CDN
- Custom domain optional (e.g., robotics-textbook.github.io or custom domain via GitHub Pages)
- Environment variables managed via GitHub repository secrets (for build-time configuration only)
- No server-side logic on book frontend (all dynamic features via Vercel API)

**API Backend (Next.js → Vercel)**:
- Next.js 14+ with App Router (API routes only, no pages)
- Deployed exclusively to Vercel (serverless functions)
- Handles: chatbot API, authentication, user progress, RAG queries
- Environment variables managed via Vercel dashboard:
  - `OPENAI_API_KEY`: OpenAI API access
  - `QDRANT_URL`, `QDRANT_API_KEY`: Vector database
  - `DATABASE_URL`: Neon Postgres connection string
  - `BETTER_AUTH_SECRET`: Auth session encryption key
  - `GITHUB_CLIENT_ID`, `GITHUB_CLIENT_SECRET`, `GOOGLE_CLIENT_ID`, `GOOGLE_CLIENT_SECRET`: OAuth
- CORS middleware configured to allow requests from GitHub Pages domain:
  ```typescript
  // middleware.ts or API route
  res.setHeader('Access-Control-Allow-Origin', 'https://username.github.io');
  res.setHeader('Access-Control-Allow-Credentials', 'true');
  res.setHeader('Access-Control-Allow-Methods', 'GET,POST,OPTIONS');
  res.setHeader('Access-Control-Allow-Headers', 'Content-Type, Authorization');
  ```

**Communication Flow**:
- ChatKit widget embedded in Docusaurus calls Vercel API routes via fetch()
- Authentication handled by Vercel API, session cookies returned with SameSite=None; Secure flags for cross-origin
- User progress tracking via Vercel API Server-Sent Events (SSE)

**Rationale**: Split platform optimizes costs (GitHub Pages free for static content, Vercel free tier for API). Static book frontend ensures fast load times and offline capability. Vercel serverless functions scale automatically for chatbot traffic. Clear separation prevents mixing concerns (content vs dynamic features).

---

### IV. Book Platform (Docusaurus 3.x + GitHub Pages)

**Rule**: Docusaurus 3.x in docs-only mode with TypeScript; flexible chapter structure (3-10 chapters).

Docusaurus configuration MUST implement:

**Core Setup**:
- Docusaurus 3.x with TypeScript configuration (`docusaurus.config.ts`)
- Docs-only mode (no blog): `presets: [['classic', { blog: false, docs: { routeBasePath: '/' } }]]`
- MDX format for chapter content (enables React component embedding)
- Flexible chapter structure: minimum 3 chapters, maximum 10 chapters
- Core topics: ROS 2, Gazebo/Unity, NVIDIA Isaac, Vision-Language-Action, hardware setup

**Styling & Theming**:
- Tailwind CSS integrated via custom Docusaurus plugin or PostCSS
- Custom component styling using Tailwind utilities in MDX
- Dark/light mode via Docusaurus built-in color mode (respects `prefers-color-scheme`)
- Prism themes for syntax highlighting (customized for robotics languages: Python, C++, YAML, XML)

**Search**:
- Built-in Docusaurus search plugin (local search) OR Algolia DocSearch (cloud search)
- Search index includes chapter titles, headings, and body content
- Search results show context snippets

**Internationalization (i18n)**:
- Default locale: English (`defaultLocale: 'en'`)
- Secondary locale: Urdu (`locales: ['en', 'ur']`)
- RTL support for Urdu: `localeConfigs: { ur: { direction: 'rtl' } }`
- Translation files: `i18n/ur/docusaurus-plugin-content-docs/current/*.md`
- Locale switcher in navbar via Docusaurus built-in component

**Chapter Metadata (Frontmatter)**:
```mdx
---
id: ros2-fundamentals
title: "Chapter 1: ROS 2 Fundamentals"
sidebar_label: "ROS 2 Fundamentals"
sidebar_position: 1
difficulty: "Beginner"
readingTime: "45 minutes"
prerequisites: ["Python basics", "Linux command line"]
---
```

**Navigation & UX**:
- Persistent sidebar showing chapter list (auto-generated from file structure)
- Difficulty indicators per chapter: Beginner, Intermediate, Advanced
- Estimated reading time displayed at chapter start
- Clear prerequisite chains: link to prerequisite chapters
- "Next Chapter" and "Previous Chapter" navigation (built-in Docusaurus feature)
- Breadcrumb navigation (built-in Docusaurus feature)
- Anchor links for all headings (shareable deep links)

**GitHub Actions Deployment**:
- Workflow file: `.github/workflows/deploy.yml`
- Trigger: push to `main` branch
- Steps: checkout, setup Node.js, install dependencies, build Docusaurus, deploy to `gh-pages` branch
- Action: `peaceiris/actions-gh-pages@v3` or equivalent

**Rationale**: Docusaurus provides battle-tested documentation platform with excellent navigation, search, and versioning. Docs-only mode removes distractions. Static generation ensures fast load times. GitHub Pages provides free, reliable hosting. MDX enables interactive components. Flexible chapter structure accommodates different course designs.

---

### V. API Backend (Next.js on Vercel)

**Rule**: Next.js 14+ App Router with API routes only; no frontend pages; CORS enabled for GitHub Pages origin.

Next.js API backend MUST implement:

**Core Configuration**:
- Next.js 14+ with App Router (`app/` directory)
- API routes only: `app/api/**/*.ts` (NO pages, NO frontend rendering)
- TypeScript strict mode enabled (`strict: true` in `tsconfig.json`)
- CORS middleware allowing GitHub Pages domain:
  ```typescript
  // middleware.ts
  import { NextResponse } from 'next/server';

  export function middleware(request: Request) {
    const origin = request.headers.get('origin');
    const allowedOrigins = [
      'https://username.github.io',
      'http://localhost:3000', // for local development
    ];

    if (allowedOrigins.includes(origin)) {
      return NextResponse.next({
        headers: {
          'Access-Control-Allow-Origin': origin,
          'Access-Control-Allow-Credentials': 'true',
          'Access-Control-Allow-Methods': 'GET,POST,OPTIONS',
          'Access-Control-Allow-Headers': 'Content-Type, Authorization',
        },
      });
    }

    return NextResponse.next();
  }

  export const config = { matcher: '/api/:path*' };
  ```

**API Routes**:
- `/api/chat/route.ts`: Chatbot streaming via SSE (POST)
- `/api/auth/*`: Better-Auth endpoints (multiple methods)
- `/api/progress/route.ts`: User reading progress (GET/POST)
- `/api/personalize/route.ts`: Content personalization (POST)

**Dependencies**:
- `@openai/agents`: OpenAI Agents JS SDK for agent orchestration
- `zod`: Runtime validation at API boundaries
- `@qdrant/js-client-rest`: Vector operations
- `@neondatabase/serverless`: Postgres queries
- `better-auth`: Authentication

**Streaming Responses**:
- Server-Sent Events (SSE) for chat responses via `TransformStream` and `ReadableStream`
- Example:
  ```typescript
  export async function POST(req: Request) {
    const stream = new TransformStream();
    const writer = stream.writable.getWriter();

    // Streaming logic here

    return new Response(stream.readable, {
      headers: {
        'Content-Type': 'text/event-stream',
        'Cache-Control': 'no-cache',
        'Connection': 'keep-alive',
      },
    });
  }
  ```

**Vercel Deployment**:
- Framework Preset: Next.js
- Build Command: `npm run build`
- Output Directory: `.next`
- Node.js Version: 18.x or higher
- Environment variables in Vercel dashboard (see Deployment Architecture section)

**Rationale**: API-only Next.js reduces deployment complexity and keeps frontend/backend separation clear. Vercel provides native Next.js optimization and automatic scaling. CORS configuration enables secure cross-origin communication. SSE streaming provides real-time chatbot responses.

---

### VI. React & TypeScript Standards

**Rule**: Functional components with hooks exclusively; TypeScript strict mode with no `any` types.

React patterns MUST follow:
- Functional components with hooks (NO class components)
- `useState` for local component state (modal open/closed, form inputs)
- `useEffect` ONLY for external system synchronization (NOT for derived state or side effects that should be in event handlers)
- `useContext` for cross-component data sharing (theme, auth, user profile)
- `useMemo` and `useCallback` for expensive computations and stable references to prevent re-renders
- `useTransition` for non-blocking state updates (search filtering, chatbot responses)
- `useDeferredValue` for deferring expensive UI updates without blocking user input
- `React.memo()` for preventing unnecessary re-renders of pure components (chapter list items, code blocks)
- Custom hooks for reusable stateful logic:
  - `useAuth()`: authentication state from Vercel API
  - `useChatSession()`: chatbot session management
  - `useReadingProgress()`: reading progress tracking
- Suspense boundaries for code-splitting with `React.lazy()` for heavy components (chart libraries, 3D visualizations)

TypeScript MUST enforce:
- Strict mode enabled in `tsconfig.json`
- No `any` types allowed (use `unknown` if type truly unknown, then narrow)
- Explicit return types for all exported functions
- Proper type inference for internal logic
- Discriminated unions for state machines (chatbot state, form wizard steps)
- Zod schemas for runtime validation at API boundaries

**Rationale**: Hooks provide cleaner state management and side-effect handling. TypeScript strict mode catches bugs at compile time, improving reliability for educational platform where downtime affects learning. `useEffect` overuse is a common anti-pattern that hurts performance.

---

### VII. Tailwind CSS Standards

**Rule**: Use Tailwind CSS for all styling; integrate with Docusaurus via custom plugin or PostCSS.

MUST use:
- Tailwind CSS for custom component styling in Docusaurus
- Custom design tokens in `tailwind.config.js` (Docusaurus requires traditional config):
  - `colors`: primary, secondary, accent, text, background
  - `fontFamily`: mono for code blocks
  - `spacing`: section, card
- Mobile-first responsive design with breakpoint prefixes (`sm:`, `md:`, `lg:`, `xl:`, `2xl:`)
- Dark mode via Tailwind `dark:` variant (synced with Docusaurus color mode via data attribute)
- Arbitrary values `[value]` sparingly, prefer theme tokens
- Integration via Docusaurus plugin or PostCSS configuration in `docusaurus.config.ts`

MUST avoid:
- `@apply` except for Docusaurus component overrides (e.g., `.markdown` class)
- Custom CSS frameworks beyond Tailwind
- Inline styles (use Tailwind utilities instead)
- Magic numbers in arbitrary values without documentation

**Rationale**: Tailwind provides utility-first CSS that scales well with component-based architecture. Mobile-first ensures accessibility on all devices. Dark mode syncing provides consistent experience. Integration with Docusaurus requires configuration but maintains Tailwind benefits.

---

### VIII. OpenAI Agents JS SDK Patterns

**Rule**: Use OpenAI Agents JS SDK for all agent orchestration; follow recommended patterns for tools and handoffs.

Agent creation MUST follow:
```typescript
import { Agent, tool } from '@openai/agents';
import { z } from 'zod';

const myAgent = new Agent({
  name: 'Book Q&A Agent',
  instructions: 'You answer questions about robotics textbook content...',
  tools: [searchBookContentTool, /* ... */],
  handoffs: [selectedTextAgent], // Optional: agents to hand off to
  model: 'gpt-4o', // Or 'gpt-4o-mini' for cost savings
});
```

Tool definition MUST follow:
```typescript
const searchBookContentTool = tool({
  name: 'search_book_content',
  description: 'Search the robotics textbook for relevant content',
  parameters: z.object({
    query: z.string().describe('Search query for textbook content'),
    chapter: z.string().optional().describe('Limit search to specific chapter'),
  }),
  execute: async ({ query, chapter }) => {
    // Hybrid search: Qdrant vector + keyword
    const results = await hybridSearch(query, chapter);
    return { results, citations: extractCitations(results) };
  },
});
```

Handoff patterns MUST implement:
- Triage agent with handoffs array: `handoffs: [bookQAAgent, selectedTextAgent]`
- Handoff decisions based on user intent detection in triage agent instructions
- No circular handoffs (A → B → A creates infinite loops)
- Each agent has clear specialization documented in instructions

Session management MUST use:
```typescript
import { OpenAIConversationsSession } from '@openai/agents';

const session = new OpenAIConversationsSession({
  store: neonPostgresStore, // Custom store implementation
  sessionId: userId,
  expiresIn: 90 * 24 * 60 * 60, // 90 days
});

const response = await agent.run({
  session,
  message: userMessage,
  streamingMode: 'full', // Enable streaming
});
```

Built-in tools MAY use:
- `webSearchTool()`: for supplementing textbook with current robotics news (use sparingly)

**Rationale**: OpenAI Agents JS SDK provides battle-tested agent orchestration with conversation memory. Tool pattern ensures type safety with Zod validation. Handoffs enable modular agent specialization. Session management prevents repeated context loading.

---

### IX. RAG Chatbot Architecture

**Rule**: Hybrid search with citations mandatory; streaming responses required for real-time feedback.

Multi-agent architecture using OpenAI Agents JS SDK MUST implement:
- **Triage Agent**: Routes user queries to appropriate specialist
  - Detects selected text context
  - Routes to Book Q&A Agent or Selected Text Agent
  - Uses `handoffs` array for routing decisions
- **Book Q&A Agent**: Answers general questions from textbook
  - Tool: `searchBookContent(query: string)` using hybrid search
  - Returns answers with chapter/section citations
  - Rejects off-topic queries politely
- **Selected Text Agent**: Answers questions about highlighted text
  - Receives selected text as context injection
  - Tool: `searchRelatedContent(selectedText: string, query: string)`
  - Explains concepts within selected passage
- All agents created with: `new Agent({ name, instructions, tools, handoffs })`
- Tools defined with: `tool({ name, description, parameters: z.object(...), execute })`
- Session management via: `OpenAIConversationsSession` with Neon Postgres backend

RAG pipeline MUST implement:
- Hybrid search: dense vectors (Qdrant `qdrant.search()`) + BM25 keyword matching
- Hierarchical chunking strategy:
  - Prose: 512 tokens with 50 token overlap
  - Code blocks: preserved whole (atomic units, never split mid-function)
  - Headings: included in chunk metadata for citation
- Vector embedding: `text-embedding-3-small` for cost efficiency
- Reranking: cross-encoder reranking of top 20 results → return top 5
- Citation requirement: every answer MUST reference source `chapter/section:line-range` format

Next.js API Routes MUST implement:
- Route Handler: `app/api/chat/route.ts` with POST method
- Server-Sent Events (SSE) streaming via `TransformStream` and `ReadableStream`
- Request validation using Zod schemas
- Error handling with structured error responses
- Session persistence in Neon Postgres using OpenAIConversationsSession
- Rate limiting per user (10 messages/minute for free tier)
- Message history retrieval from Postgres on session resume

**Rationale**: RAG ensures chatbot answers are grounded in textbook content, preventing hallucinations. Multi-agent architecture enables specialized handling (general Q&A vs selected text). Citations enable verification and deepen learning. Streaming provides immediate feedback for better UX. OpenAI Agents JS SDK provides robust agent orchestration with built-in conversation memory.

---

### X. Chatbot Frontend (ChatKit in Docusaurus)

**Rule**: ChatKit embedded as custom Docusaurus component; wrapped in BrowserOnly to avoid SSR issues.

ChatKit integration MUST implement:

**Component Structure**:
```tsx
// src/components/ChatbotWidget.tsx
import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';

export default function ChatbotWidget() {
  return (
    <BrowserOnly fallback={<div>Loading chatbot...</div>}>
      {() => {
        const { ChatKit } = require('@openai/chatkit'); // Dynamic import
        return (
          <ChatKit
            apiEndpoint="https://your-vercel-api.vercel.app/api/chat"
            authEndpoint="https://your-vercel-api.vercel.app/api/auth/session"
            onError={(error) => console.error('ChatKit error:', error)}
            enableStreaming={true}
            placeholder="Ask about robotics concepts..."
          />
        );
      }}
    </BrowserOnly>
  );
}
```

**Features**:
- Streaming responses with typing indicator
- Message history for authenticated users (retrieved from Vercel API)
- Selected text feature:
  - Detect `window.getSelection()` on text selection
  - Show floating "Ask about this" button using portal
  - Inject selected text as context into chat API request
- Error handling with retry mechanism using React Error Boundary
- Citation display linking to chapter sections (parse response, convert to Docusaurus internal links)
- Loading states with skeleton UI during streaming
- Message retry button for failed requests

**Docusaurus Integration**:
- Register component in `docusaurus.config.ts` theme config or use swizzling
- Add to theme layout (e.g., floating button bottom-right, or sidebar panel)
- Ensure component only renders client-side (no SSR) via `BrowserOnly`

**API Communication**:
- Fetch calls to Vercel API endpoints with credentials included:
  ```typescript
  fetch('https://your-api.vercel.app/api/chat', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    credentials: 'include', // Include cookies for auth
    body: JSON.stringify({ message, sessionId }),
  })
  ```
- Handle CORS with credentials: API must return `Access-Control-Allow-Credentials: true`

**Rationale**: ChatKit provides production-ready chat UI. BrowserOnly prevents SSR errors with browser-dependent APIs. Streaming provides real-time feedback. Selected text feature enables context-aware questioning. Citation links deepen learning by connecting answers to source material.

---

### XI. Authentication & Content Personalization

**Rule**: Better-Auth with OAuth providers; secure session management with cross-origin cookies.

Authentication MUST implement:
- OAuth providers: Google and GitHub minimum (configured in Better-Auth)
- Auth endpoints on Vercel: `/api/auth/*`
- Session cookies with flags: `httpOnly: true`, `secure: true`, `sameSite: 'none'` (for cross-origin)
- Session expiration: 30 days inactive, 90 days absolute
- User profiles stored in Neon Postgres with schema:
  ```sql
  CREATE TABLE users (
    id UUID PRIMARY KEY,
    email TEXT UNIQUE NOT NULL,
    name TEXT,
    avatar_url TEXT,
    python_level TEXT CHECK (python_level IN ('beginner', 'intermediate', 'advanced')),
    ros_experience BOOLEAN,
    hardware_access BOOLEAN,
    learning_goals TEXT[],
    created_at TIMESTAMP DEFAULT NOW()
  );
  ```

Onboarding questionnaire (authenticated users) MUST capture:
- Python level: Beginner / Intermediate / Advanced
- ROS experience: Yes / No
- Hardware access: Yes (specify equipment) / No (simulation only)
- Learning goals: Career change / Academic research / Hobby / Professional upskilling

Content personalization MUST provide:
- "Personalize" button at chapter start (custom Docusaurus component)
- Button calls Vercel API: `POST /api/personalize` with user profile
- API returns adapted content suggestions or dynamic instructions
- Chatbot instructions dynamically include user background in agent system prompt:
  ```typescript
  const instructions = `You are a robotics tutor. The user has ${user.python_level} Python
  experience and ${user.ros_experience ? 'has' : 'has no'} ROS experience. Adapt your
  explanations accordingly.`;
  ```
- Docusaurus components check auth state via API call: `GET /api/auth/session`

**Rationale**: OAuth reduces friction compared to email/password. Secure cross-origin sessions enable split platform architecture. Personalization improves learning outcomes by adapting content to user background. Better-Auth provides superior TypeScript support compared to NextAuth.

---

### XII. Internationalization (Docusaurus i18n)

**Rule**: English default, Urdu RTL fully supported; technical terms remain English.

Docusaurus i18n MUST implement:

**Configuration**:
```typescript
// docusaurus.config.ts
export default {
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      en: { label: 'English', direction: 'ltr' },
      ur: { label: 'اردو', direction: 'rtl' },
    },
  },
  // ...
};
```

**Translation Files**:
- Docs: `i18n/ur/docusaurus-plugin-content-docs/current/*.md`
- UI strings: `i18n/ur/docusaurus-theme-classic/*.json`
- Navbar: `i18n/ur/docusaurus-theme-classic/navbar.json`
- Footer: `i18n/ur/docusaurus-theme-classic/footer.json`

**Technical Terms Handling**:
- Technical terms maintain English with Urdu explanation in parentheses:
  - English: "The ROS node publishes to the topic"
  - Urdu: "ROS node (نوڈ) topic (موضوع) پر publish کرتا ہے"
- Code comments remain in English (universal programming language convention)

**Locale Switching**:
- Docusaurus built-in locale switcher in navbar
- Locale routing: `/` (English), `/ur/` (Urdu)

**RTL Support**:
- Automatic RTL layout when `direction: 'rtl'` set in localeConfigs
- Tailwind RTL plugin ensures custom components respect RTL

**Translation Process**:
- Urdu translations require technical review before publication
- Translation button at chapter start switches locale via Docusaurus routing

**Rationale**: RTL support enables access for Urdu-speaking learners (large underserved market). Maintaining English technical terms prevents translation ambiguity in code. Docusaurus i18n provides built-in routing and fallback handling.

---

### XIII. Frontend Reading Experience

**Rule**: Typography optimized for technical content; persistent navigation; keyboard shortcuts enabled.

Reading experience MUST provide:
- Typography optimized for technical content:
  - 18px base font size for body text
  - 1.6 line height for readability
  - Max 75ch line length to prevent eye strain
  - `font-mono` for all code and technical terms
- Syntax-highlighted code blocks with:
  - Copy button with visual feedback (built-in Docusaurus feature)
  - Line numbers for referencing (Prism plugin)
  - Language label badge
  - Horizontal scroll on mobile (NOT text wrap which breaks indentation)
- Dark/light mode toggle using Docusaurus built-in color mode, persisted in localStorage
- Adjustable font size (small/medium/large) via custom Docusaurus component, persisted in localStorage
- Expandable code examples: custom MDX component showing simplified version initially, revealing full on click
- Back-to-top button on pages longer than 2 viewports (custom Docusaurus plugin or component)
- Persistent sidebar showing chapter list and current scroll position indicator (built-in Docusaurus feature)

Navigation MUST include:
- Breadcrumb navigation (built-in Docusaurus feature)
- Full-text search powered by Docusaurus search plugin or Algolia DocSearch
- "Related Topics" suggestions at end of each section (custom MDX component)
- "Next Chapter" and "Previous Chapter" navigation (built-in Docusaurus feature with auto prefetching)
- Anchor links for all headings (shareable deep links, built-in Docusaurus feature)
- Table of contents sidebar for current page (built-in Docusaurus feature)
- Keyboard shortcuts: `/` for search, `n` for next chapter, `p` for previous (custom Docusaurus plugin)

**Rationale**: Educational platforms must provide optimal reading experience for technical content. Docusaurus provides many features out-of-box. Custom components extend functionality. Keyboard shortcuts improve power user experience. Persistent sidebar aids navigation.

---

### XIV. User Experience & Accessibility

**Rule**: WCAG AA compliance mandatory; Core Web Vitals thresholds must pass.

Accessibility (WCAG AA) MUST enforce:
- All images have descriptive alt text (NOT decorative "image" or empty)
- Color contrast ratio minimum 4.5:1 for normal text, 3:1 for large text
- Full keyboard navigation for all interactive elements (tab order logical)
- Focus indicators using Tailwind `focus:ring-2` and `focus-visible:` variants
- Proper heading hierarchy (h1 > h2 > h3, no level skips) validated in CI
- ARIA labels for custom interactive components (chatbot, code copy buttons)
- No information conveyed by color alone (use icons + color)
- RTL layout fully functional for Urdu content using Tailwind RTL plugin (`dir="rtl"`)
- Skip to main content link for keyboard users (Docusaurus provides this)

Performance (Core Web Vitals) MUST meet:
- Largest Contentful Paint (LCP): under 2.5 seconds
- First Input Delay (FID): under 100 milliseconds
- Cumulative Layout Shift (CLS): under 0.1
- Docusaurus static generation for fast loads
- Image optimization: lazy load images below fold
- Code-split heavy components (chart libraries, 3D visualizations) using React.lazy()
- Target < 200KB initial JS bundle

Responsive design MUST implement:
- Mobile-first CSS approach using Tailwind breakpoints
- Breakpoints: default (mobile), `sm:640px`, `md:768px`, `lg:1024px`, `xl:1280px`, `2xl:1536px`
- Code blocks horizontally scrollable on mobile using `overflow-x-auto` (NOT responsive text size)
- Collapsible sidebar on tablet/mobile (built-in Docusaurus feature)
- Touch targets minimum 44x44px using Tailwind sizing utilities (`min-h-11 min-w-11`)
- Images responsive using optimized image plugin
- Tables scrollable horizontally on small screens using `overflow-x-auto` wrapper

**Rationale**: Educational platforms must be accessible to all learners regardless of ability or device. Performance affects learning retention and engagement. Technical readers expect monospace fonts and precise code formatting. WCAG AA is non-negotiable for inclusive education.

---

### XV. Book Structure & Content Quality

**Rule**: Flexible chapter structure (3-10 chapters) with strict quality standards for each chapter.

Book structure MUST provide:
- MDX format content (enables React component embedding in Docusaurus)
- Flexible chapter count: minimum 3 chapters, maximum 10 chapters
- Core topics coverage:
  - ROS 2 fundamentals and advanced patterns
  - Gazebo/Unity simulation environments
  - NVIDIA Isaac Sim for robot simulation
  - Vision-Language-Action (VLA) models
  - Hardware setup and safety (electrical, mechanical)
- Chapter metadata in frontmatter:
  ```mdx
  ---
  id: ros2-fundamentals
  title: "Chapter 1: ROS 2 Fundamentals"
  sidebar_label: "ROS 2 Fundamentals"
  sidebar_position: 1
  difficulty: "Beginner"
  readingTime: "45 minutes"
  prerequisites: ["Python basics", "Linux command line"]
  ---
  ```
- Clear prerequisite chains: prerequisite graph showing chapter dependencies
- Difficulty indicators per chapter: Beginner, Intermediate, Advanced
- Estimated reading time displayed prominently at chapter start

Chapter content MUST include:
- **Learning Objectives**: 3-5 specific, measurable outcomes at chapter start
- **Prerequisites**: explicit list with links to prerequisite chapters or external resources
- **Quick Start**: 15-minute path to get something working (immediate gratification)
- **Deep Dive**: comprehensive explanation with theory and edge cases
- **Code Examples**: all runnable with exact version specifications in comments
- **Architecture Diagrams**: Mermaid or image files (NEVER rely on prose alone for system architecture)
- **Common Errors**: troubleshooting section with error messages and solutions
- **Safety Warnings**: for hardware-related content (robot movement, electrical work)
- **Further Reading**: curated list of official docs, papers, tutorials
- **Exercises**: hands-on challenges with solution links (collapsed by default using MDX details component)

Version pinning MUST specify:
- ROS 2 Humble (LTS release)
- Isaac Sim 2023.1.1 (or latest tested version)
- Python 3.11 for robotics code examples
- All Python packages pinned in `requirements.txt` within code block
- All npm packages pinned in `package.json` within code block

**Rationale**: Flexible chapter structure accommodates different course designs (3-chapter intro course or 10-chapter comprehensive program). Strict quality standards ensure every chapter delivers value. Version pinning prevents breakage. MDX enables rich interactive components. Docusaurus provides excellent documentation platform.

---

### XVI. Performance & Quality Gates

**Rule**: No feature ships without passing all quality gates.

MUST pass before deployment:
- **TypeScript**: Strict mode with no `any` types, builds without errors
- **Linting**: ESLint + Prettier enforced, zero warnings
- **Tests**: 80% coverage minimum for API routes (Next.js Route Handlers)
- **RAG Precision**: > 0.85 on test query set (50 representative questions with known correct answers)
- **Accessibility**: WCAG AA audit pass using axe DevTools or Lighthouse
- **Performance**: Lighthouse score > 90 (Performance, Accessibility, Best Practices, SEO)
- **Code Examples**: All examples pass automated tests in CI (extract and run examples)
- **Cross-browser**: Manual testing on Chrome, Firefox, Safari, Edge
- **Mobile**: Manual testing on iOS Safari, Android Chrome
- **Chapter Completeness**: No chapter ships without all required sections (objectives, prerequisites, diagrams, common errors)

Automated checks MUST run in CI:
- `npm run type-check`: TypeScript compilation
- `npm run lint`: ESLint + Prettier
- `npm test`: Jest/Vitest tests for API routes
- `npm run test:e2e`: Playwright tests for critical user flows (auth, chat, reading progress)
- `npm run test:rag`: RAG precision evaluation script
- `npm run test:examples`: Extract and run all code examples from MDX
- Lighthouse CI for performance regression detection

Quality metrics MUST track:
- Code coverage trend (require improvement or maintenance, never decline)
- RAG precision trend on test set
- Lighthouse score trend
- Build time trend (flag if > 5 minutes)
- Bundle size trend (flag if client JS > 250KB)

**Rationale**: Educational platforms demand high reliability. Broken code examples damage trust and learning outcomes. Accessibility is non-negotiable for inclusive education. Performance affects learning engagement. Automated gates prevent regressions.

---

## Technology Stack Constraints

**Non-Negotiable Technology Choices**:

Book Frontend:
- Docusaurus 3.x
- React 18+ (Docusaurus dependency)
- TypeScript 5+
- Tailwind CSS for custom component styling
- MDX for chapter content

API Backend:
- Node.js 18+ (for Next.js compatibility)
- Next.js 14+ with App Router (API routes only, no pages)
- TypeScript 5+ strict mode
- OpenAI Agents JS SDK (@openai/agents) for agent orchestration
- OpenAI ChatKit for chat UI component
- Zod v3 for parameter validation
- @qdrant/js-client-rest for vector operations
- @neondatabase/serverless for Postgres
- Better-Auth for authentication

Deployment:
- Book: GitHub Pages via GitHub Actions
- API: Vercel exclusively

Architecture Patterns:
- No premature abstractions - use framework features directly
- Prefer React functional components with hooks
- Use Tailwind utilities directly, avoid `@apply` except for Docusaurus overrides
- NO custom CSS frameworks - Tailwind only
- NO unnecessary service layers - keep logic in API Route Handlers
- Split platform: static book frontend + dynamic API backend

**Rationale**: Stack constraints prevent bikeshedding, ensure team expertise concentration, and leverage modern best practices. Split platform optimizes costs (GitHub Pages free for static, Vercel free tier for API). Docusaurus provides battle-tested documentation platform. OpenAI Agents JS SDK provides robust agent orchestration. Free tiers enable low-cost deployment for educational projects.

---

## Development Workflow

**Execution Contract for Every Feature**:

1. Confirm surface and success criteria (one sentence)
2. List constraints, invariants, non-goals
3. Produce artifact with acceptance checks inlined (checkboxes or tests)
4. Add follow-ups and risks (max 3 bullets)
5. Create PHR in appropriate subdirectory under `history/prompts/` (constitution, feature-name, or general)
6. If plan/tasks identified significant decisions, suggest ADR documentation

**Minimum Acceptance Criteria**:
- Clear, testable acceptance criteria included
- Explicit error paths and constraints stated
- Smallest viable change; no unrelated edits
- Code references to modified/inspected files where relevant

**Human as Tool Strategy**:

Invoke user for input when encountering situations requiring human judgment:

1. **Ambiguous Requirements**: Ask 2-3 targeted clarifying questions before proceeding
2. **Unforeseen Dependencies**: Surface dependencies and ask for prioritization
3. **Architectural Uncertainty**: Present options with tradeoffs, get user preference
4. **Completion Checkpoint**: Summarize what was done and confirm next steps

**Default Policies**:
- Clarify and plan first - separate business understanding from technical plan
- Do not invent APIs, data, or contracts; ask targeted clarifiers if missing
- Never hardcode secrets or tokens; use `.env` and document in README
- Prefer smallest viable diff; do not refactor unrelated code
- Cite existing code with code references (`file:line-range`); propose new code in fenced blocks
- Keep reasoning private; output only decisions, artifacts, and justifications

**Code Standards**:
- Functional programming patterns preferred (pure functions, immutability)
- Error handling via Result types or explicit error returns (NOT uncaught exceptions)
- Logging structured with context (user ID, request ID, operation)
- Comments explain "why" not "what" (code should be self-documenting)
- Test names follow: `it('should [expected behavior] when [condition]')`

---

## Governance

**Constitutional Authority**:

This constitution supersedes all other practices. All PRs, code reviews, and design decisions MUST verify compliance with stated principles.

**Amendment Procedure**:

1. Propose amendment with rationale addressing "Why Needed" and "Simpler Alternative Rejected Because"
2. Document amendment in ADR if architecturally significant
3. Update `.specify/memory/constitution.md` with incremented version following semantic versioning
4. Update sync impact report at top of constitution file
5. Propagate changes to all dependent templates:
   - `.specify/templates/plan-template.md`
   - `.specify/templates/spec-template.md`
   - `.specify/templates/tasks-template.md`
6. Create migration plan for affected in-flight features

**Version Semantics**:

- **MAJOR (X.0.0)**: Backward incompatible governance/principle removals or redefinitions requiring architectural changes
- **MINOR (x.Y.0)**: New principle/section added or materially expanded guidance without breaking changes
- **PATCH (x.y.Z)**: Clarifications, wording improvements, typo fixes, non-semantic refinements

**Complexity Justification**:

Any violation of constitutional principles MUST be justified in the "Complexity Tracking" section of `plan.md` with:
- What principle is violated
- Why it's needed for this specific feature
- What simpler alternative was rejected and why

**Compliance Review**:

- Constitution check mandatory before Phase 0 research in planning
- Re-check after Phase 1 design before task generation
- Automated linting for quality gates (tests, accessibility, performance) in CI
- Manual review for architectural decisions and complexity justifications
- No PR approved without constitution compliance verification

**Runtime Guidance**:

For day-to-day development instructions, see `CLAUDE.md` (agent-specific) and template command files in `.claude/commands/sp.*.md`.

---

**Version**: 3.0.0 | **Ratified**: 2025-12-02 | **Last Amended**: 2025-12-02
