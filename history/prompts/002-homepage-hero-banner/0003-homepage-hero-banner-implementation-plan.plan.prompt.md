---
id: 0003
title: homepage hero banner implementation plan
stage: plan
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: 002-homepage-hero-banner
branch: 002-homepage-hero-banner
user: Irfan Ahmed
command: /sp.plan
labels: ["plan", "docusaurus", "hero-banner", "swizzling", "responsive-design"]
links:
  spec: specs/002-homepage-hero-banner/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/002-homepage-hero-banner/plan.md
  - specs/002-homepage-hero-banner/research.md
  - specs/002-homepage-hero-banner/data-model.md
  - specs/002-homepage-hero-banner/quickstart.md
  - CLAUDE.md (agent context updated)
tests:
  - Constitution check (all principles validated)
  - Quickstart testing guide created
---

## Prompt

/sp.plan lets update the current doscusaurus book with a landing homepage, use the logo and help banner [user provided two images]

## Response snapshot

Created comprehensive implementation plan for homepage hero banner feature using Docusaurus component swizzling.

**Architecture Decisions:**

1. **Component Swizzling (Wrap Mode)**:
   - Swizzle DocItem/Layout to inject hero above intro.md content
   - Homepage detection: `metadata.slug === '/' || metadata.id === 'intro'`
   - Preserves docs-only mode architecture

2. **Styling Approach**:
   - CSS Modules (zero dependencies, built-in Docusaurus)
   - Justified deviation from Tailwind CSS (constitution exception documented)
   - Scoped class names prevent conflicts

3. **Image Optimization**:
   - WebP primary format (60-80% smaller than JPEG)
   - JPEG fallback for older browsers
   - Target: <200KB for hero banner, <50KB for logo

4. **Responsive Design**:
   - Docusaurus standard breakpoints: 996px (desktop), 768px (tablet), 576px (mobile)
   - Mobile-first CSS with media queries
   - CTA buttons horizontal → vertical on mobile

5. **Accessibility**:
   - Semantic HTML (`<header>`, `<h1>`, `<nav>`)
   - ARIA labels for screen readers
   - Contrast ratio 4.5:1 (WCAG AA)
   - Keyboard navigation via Docusaurus Link component

**Artifacts Created:**

- `plan.md`: Full implementation design with architecture decisions, risk assessment, acceptance criteria
- `research.md`: Docusaurus swizzling patterns, static asset management, responsive design research
- `data-model.md`: Component data structures, static assets, validation rules
- `quickstart.md`: 10-15 minute testing guide with visual checklist, responsive testing, accessibility audits

**Key Technical Details:**

- Language: TypeScript 5+, Node.js 18+
- Framework: Docusaurus 3.9.2 (existing)
- Styling: CSS Modules (scoped)
- Routing: Docusaurus Link (SPA navigation)
- Assets: `static/img/hero-banner.webp`, `static/img/logo.png`
- Performance: LCP <2.5s, page load <2s

**Constitution Compliance:**

- ✅ Documentation-First: Consulted Docusaurus 3.x docs via Context7 MCP
- ✅ React & TypeScript Standards: Functional component with hooks, strict mode
- ⚠️ Tailwind CSS: Exception granted (CSS Modules simpler for single component)
- ✅ Accessibility: WCAG AA compliant
- ✅ Performance: Lighthouse > 90 targets

## Outcome

- ✅ Impact: Complete implementation plan ready for task generation
- 🧪 Tests: Quickstart guide with 8 test scenarios (visual, responsive, accessibility, performance)
- 📁 Files: plan.md, research.md, data-model.md, quickstart.md, CLAUDE.md (context updated)
- 🔁 Next prompts: /sp.tasks to generate implementation tasks
- 🧠 Reflection: Component swizzling in wrap mode is safest approach for Docusaurus customization; CSS Modules justified for single-component scope; WebP optimization critical for performance

## Evaluation notes (flywheel)

- Failure modes observed: None - all constitution gates passed except justified Tailwind deviation
- Graders run and results (PASS/FAIL): Constitution check PASS (14/15 gates, 1 justified exception)
- Prompt variant (if applicable): Standard /sp.plan workflow with Phase 0 research agent
- Next experiment (smallest change to try): Proceed to /sp.tasks for task breakdown
