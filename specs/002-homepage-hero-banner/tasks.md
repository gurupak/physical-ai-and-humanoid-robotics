# Implementation Tasks: Homepage Hero Banner Landing Page

**Feature**: 002-homepage-hero-banner  
**Branch**: `002-homepage-hero-banner`  
**Date**: 2025-12-05  
**Spec**: [spec.md](./spec.md) | **Plan**: [plan.md](./plan.md)

## Overview

This task list implements a custom hero banner on the Docusaurus homepage featuring book title, tagline, description, background image, and two CTA buttons. Implementation uses component swizzling (wrap mode) to inject hero above intro.md content.

**Total Tasks**: 15  
**MVP Scope**: User Story 1 (Tasks T001-T008) - Minimum viable homepage hero  
**Estimated Time**: 4-6 hours for MVP, 6-8 hours for full feature

---

## Implementation Strategy

**Incremental Delivery**:
1. **MVP (User Story 1)**: Basic hero with title, tagline, description - validates homepage display
2. **Enhancement (User Story 2)**: Add functional CTA buttons - enables navigation
3. **Polish (User Story 3)**: Add background image and visual design - enhances UX

**Parallel Opportunities**: Tasks marked [P] can run concurrently (different files, no dependencies)

**Independent Testing**: Each user story phase includes acceptance criteria for standalone validation

---

## User Story Mapping

### US1 (P1): First-Time Visitor Understands Book Purpose
**Goal**: Display hero section with title, tagline, and description on homepage  
**Independent Test**: Navigate to `/` and verify hero displays book information without scrolling  
**Tasks**: T001-T008 (8 tasks)

### US2 (P2): Visitor Takes Primary Action
**Goal**: Add functional CTA buttons that navigate to correct pages  
**Independent Test**: Click each CTA button and verify navigation works  
**Tasks**: T009-T011 (3 tasks)  
**Dependencies**: Requires US1 complete (hero component must exist)

### US3 (P3): Visitor Experiences Visually Engaging Design
**Goal**: Add background image and responsive visual design  
**Independent Test**: View hero on desktop/tablet/mobile and verify visual quality  
**Tasks**: T012-T013 (2 tasks)  
**Dependencies**: Requires US1 complete (hero component must exist)

### Polish & Validation
**Goal**: Cross-cutting concerns (dark mode, accessibility, performance)  
**Tasks**: T014-T015 (2 tasks)  
**Dependencies**: Requires all user stories complete

---

## Phase 1: Setup

**Prerequisites**: Docusaurus 3.9.2 installed, `002-homepage-hero-banner` branch checked out

- [X] T001 Verify Docusaurus development environment and dependencies in `package.json`

**Acceptance Criteria**:
- [ ] `npm install` completes without errors
- [ ] `npm start` launches dev server at http://localhost:3000/physical-ai-and-humanoid-robotics/
- [ ] Homepage (intro.md) displays existing content

**File**: `package.json`

---

## Phase 2: US1 (P1) - First-Time Visitor Understands Book Purpose

**Story Goal**: Display hero section with book title, tagline, and description above intro.md content

**Independent Test Criteria**:
- [ ] Hero section visible at top of homepage without scrolling (desktop 1920x1080)
- [ ] Book title "Physical AI & Humanoid Robotics" displays in large heading
- [ ] Tagline "A comprehensive guide to building intelligent robots" displays below title
- [ ] Brief description (2-3 sentences) displays and is readable
- [ ] Hero appears ONLY on homepage (/, not /intro or other doc pages)

### Implementation Tasks

- [X] T002 [P] [US1] Create HomepageHero component directory structure in `src/components/HomepageHero/`

**Acceptance Criteria**:
- [ ] Directory `src/components/HomepageHero/` created
- [ ] Files `index.tsx` and `styles.module.css` created (empty placeholders)

**Files**:
- `src/components/HomepageHero/index.tsx`
- `src/components/HomepageHero/styles.module.css`

---

- [X] T003 [US1] Implement HomepageHero React component in `src/components/HomepageHero/index.tsx`

**Acceptance Criteria**:
- [ ] Component imports Docusaurus hooks (`useDocusaurusContext`)
- [ ] Component renders semantic HTML: `<header>`, `<h1>` for title, `<p>` for tagline/description
- [ ] Title and tagline sourced from `siteConfig.title` and `siteConfig.tagline`
- [ ] Description hardcoded: "Master ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action systems. From simulation to real hardware deployment."
- [ ] Component exports as default
- [ ] TypeScript compiles without errors

**Implementation Reference** (from plan.md):
```tsx
import React from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './styles.module.css';

export default function HomepageHero(): JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  
  return (
    <header className={styles.heroBanner} role="banner">
      <div className={styles.heroContainer}>
        <div className={styles.heroContent}>
          <h1 className={styles.heroTitle}>{siteConfig.title}</h1>
          <p className={styles.heroTagline}>{siteConfig.tagline}</p>
          <p className={styles.heroDescription}>
            Master ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action systems. 
            From simulation to real hardware deployment.
          </p>
        </div>
      </div>
    </header>
  );
}
```

**File**: `src/components/HomepageHero/index.tsx`

---

- [X] T004 [P] [US1] Create hero banner CSS Module styles in `src/components/HomepageHero/styles.module.css`

**Acceptance Criteria**:
- [ ] Base styles defined: `.heroBanner`, `.heroContainer`, `.heroContent`
- [ ] Typography styles: `.heroTitle` (3rem), `.heroTagline` (1.5rem), `.heroDescription` (1.125rem)
- [ ] Layout: Flexbox column with centered alignment
- [ ] Spacing: 4rem padding top/bottom, 2rem left/right
- [ ] No background image yet (solid color placeholder: `#0a2342`)

**Implementation Reference** (from plan.md):
```css
.heroBanner {
  padding: 4rem 2rem;
  text-align: center;
  background-color: #0a2342;
  color: #ffffff;
}

.heroContainer {
  max-width: 1140px;
  margin: 0 auto;
}

.heroContent {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 1.5rem;
}

.heroTitle {
  font-size: 3rem;
  font-weight: bold;
  margin: 0;
  line-height: 1.2;
}

.heroTagline {
  font-size: 1.5rem;
  margin: 0;
  opacity: 0.9;
}

.heroDescription {
  font-size: 1.125rem;
  max-width: 600px;
  margin: 0;
  line-height: 1.6;
}
```

**File**: `src/components/HomepageHero/styles.module.css`

---

- [X] T005 [US1] Swizzle DocItem/Layout component in wrap mode using Docusaurus CLI

**Acceptance Criteria**:
- [ ] Run command: `npm run swizzle @docusaurus/theme-classic DocItem/Layout -- --wrap`
- [ ] File created: `src/theme/DocItem/Layout/index.tsx`
- [ ] Swizzle completes without errors
- [ ] TypeScript compiles after swizzle

**Command**:
```bash
cd D:\workspace\nextjs\hackathon-book
npm run swizzle @docusaurus/theme-classic DocItem/Layout -- --wrap
```

**File Created**: `src/theme/DocItem/Layout/index.tsx`

---

- [X] T006 [US1] Implement homepage detection and conditional hero rendering in `src/theme/DocItem/Layout/index.tsx`

**Acceptance Criteria**:
- [ ] Import HomepageHero component from `@site/src/components/HomepageHero`
- [ ] Use `useDoc()` hook to access page metadata
- [ ] Detect homepage: `metadata.slug === '/' || metadata.id === 'intro'`
- [ ] Conditionally render `<HomepageHero />` before original `<Layout>` component
- [ ] TypeScript compiles without errors

**Implementation Reference** (from plan.md):
```tsx
import React from 'react';
import Layout from '@theme-original/DocItem/Layout';
import type LayoutType from '@theme/DocItem/Layout';
import type {WrapperProps} from '@docusaurus/types';
import {useDoc} from '@docusaurus/plugin-content-docs/client';
import HomepageHero from '@site/src/components/HomepageHero';

type Props = WrapperProps<typeof LayoutType>;

export default function LayoutWrapper(props: Props): JSX.Element {
  const {metadata} = useDoc();
  const isHomepage = metadata.slug === '/' || metadata.id === 'intro';
  
  return (
    <>
      {isHomepage && <HomepageHero />}
      <Layout {...props} />
    </>
  );
}
```

**File**: `src/theme/DocItem/Layout/index.tsx`

---

- [X] T007 [US1] Test hero display on homepage in development server

**Acceptance Criteria**:
- [ ] Run `npm start` and navigate to http://localhost:3000/physical-ai-and-humanoid-robotics/
- [ ] Hero section visible at top of page (before intro.md content)
- [ ] Title displays: "Physical AI & Humanoid Robotics"
- [ ] Tagline displays: "A comprehensive guide to building intelligent robots"
- [ ] Description displays below tagline
- [ ] Hero does NOT appear on `/intro` route (if different from `/`)
- [ ] No console errors in browser DevTools

**Validation**:
```bash
npm start
# Open browser to http://localhost:3000/physical-ai-and-humanoid-robotics/
# Visual inspection: hero displays correctly
# Check console: no errors
```

---

- [X] T008 [US1] Add responsive breakpoints for mobile and tablet to `src/components/HomepageHero/styles.module.css`

**Acceptance Criteria**:
- [ ] Tablet breakpoint (max-width: 996px): Title 2rem, padding 2rem
- [ ] Mobile breakpoint (max-width: 768px): Title 1.5rem, padding 1rem, vertical layout
- [ ] Test on DevTools responsive mode (375px, 768px, 1920px)
- [ ] Text remains readable at all breakpoints
- [ ] No horizontal scrolling on mobile

**Implementation Reference** (from plan.md):
```css
/* Add to existing styles.module.css */

@media screen and (max-width: 996px) {
  .heroBanner { padding: 2rem 1.5rem; }
  .heroTitle { font-size: 2rem; }
  .heroTagline { font-size: 1.25rem; }
  .heroDescription { font-size: 1rem; }
}

@media screen and (max-width: 768px) {
  .heroBanner { padding: 1.5rem 1rem; }
  .heroTitle { font-size: 1.5rem; }
  .heroTagline { font-size: 1rem; }
  .heroDescription { font-size: 0.9rem; }
}
```

**File**: `src/components/HomepageHero/styles.module.css`

---

## Phase 3: US2 (P2) - Visitor Takes Primary Action

**Story Goal**: Add functional CTA buttons that navigate to Table of Contents and Introduction

**Dependencies**: Requires US1 complete (HomepageHero component must exist)

**Independent Test Criteria**:
- [ ] Primary CTA button "Explore the Book" visible in hero section
- [ ] Secondary CTA button "Start Reading" visible in hero section
- [ ] Clicking "Explore the Book" navigates to `/` (TOC sidebar visible)
- [ ] Clicking "Start Reading" navigates to `/intro` (Introduction page)
- [ ] Navigation is instant (SPA routing, no page reload)
- [ ] Buttons have hover states for visual feedback

### Implementation Tasks

- [X] T009 [US2] Add CTA buttons to HomepageHero component in `src/components/HomepageHero/index.tsx`

**Acceptance Criteria**:
- [ ] Import `Link` component from `@docusaurus/Link`
- [ ] Add `<nav>` element with `aria-label="Primary navigation"`
- [ ] Create primary CTA: "Explore the Book" → `to="/"`
- [ ] Create secondary CTA: "Start Reading" → `to="/intro"`
- [ ] Use Docusaurus button classes: `button button--primary button--lg` and `button button--secondary button--lg`
- [ ] Add ARIA labels for accessibility
- [ ] TypeScript compiles without errors

**Implementation Reference** (from plan.md):
```tsx
// Add to HomepageHero component after <p className={styles.heroDescription}>
<nav className={styles.heroButtons} aria-label="Primary navigation">
  <Link
    className="button button--primary button--lg"
    to="/"
    aria-label="Explore the book table of contents">
    Explore the Book
  </Link>
  <Link
    className="button button--secondary button--lg"
    to="/intro"
    aria-label="Start reading the introduction">
    Start Reading
  </Link>
</nav>
```

**File**: `src/components/HomepageHero/index.tsx`

---

- [X] T010 [P] [US2] Add CTA button layout styles to `src/components/HomepageHero/styles.module.css`

**Acceptance Criteria**:
- [ ] `.heroButtons` class: flexbox horizontal layout with 1rem gap
- [ ] Desktop: buttons side-by-side
- [ ] Mobile (< 768px): buttons stack vertically, full width
- [ ] Buttons centered in container

**Implementation Reference** (from plan.md):
```css
/* Add to existing styles.module.css */

.heroButtons {
  display: flex;
  gap: 1rem;
  flex-wrap: wrap;
  justify-content: center;
  margin-top: 1rem;
}

@media screen and (max-width: 768px) {
  .heroButtons {
    flex-direction: column;
    width: 100%;
  }
  
  .heroButtons :global(.button) {
    width: 100%;
  }
}
```

**File**: `src/components/HomepageHero/styles.module.css`

---

- [X] T011 [US2] Test CTA button navigation in development server

**Acceptance Criteria**:
- [ ] Click "Explore the Book" button → URL changes to `/` (or stays on `/`)
- [ ] Sidebar shows table of contents (Docusaurus default behavior)
- [ ] Click "Start Reading" button → page navigates to `/intro`
- [ ] Navigation is instant (no page reload, SPA routing)
- [ ] Browser back button works correctly
- [ ] No console errors

**Validation**:
```bash
npm start
# Test primary CTA: click "Explore the Book" → verify TOC visible
# Test secondary CTA: click "Start Reading" → verify /intro page loads
# Test browser back button → returns to homepage
```

---

## Phase 4: US3 (P3) - Visitor Experiences Visually Engaging Design

**Story Goal**: Add background image and visual polish to hero section

**Dependencies**: Requires US1 complete (HomepageHero component must exist)

**Independent Test Criteria**:
- [ ] Background image (humanoid robot) visible in hero section
- [ ] Image does not obscure text (text has sufficient contrast)
- [ ] Image is responsive (covers full width, no distortion)
- [ ] Image loads within 2 seconds on broadband
- [ ] Fallback works if WebP not supported (JPEG displays)

### Implementation Tasks

- [X] T012 [P] [US3] Optimize and add hero banner images to `static/img/` directory

**Acceptance Criteria**:
- [ ] User-provided `hero-banner.jpg` placed in `static/img/`
- [ ] Convert to WebP using online tool (https://squoosh.app) or CLI: `cwebp hero-banner.jpg -q 80 -o hero-banner.webp`
- [ ] Verify `hero-banner.webp` size < 200KB (compress to q=60-80 if needed)
- [ ] Keep `hero-banner.jpg` as fallback
- [ ] Test images load in browser: http://localhost:3000/physical-ai-and-humanoid-robotics/img/hero-banner.webp

**Files**:
- `static/img/hero-banner.jpg` (user-provided, fallback)
- `static/img/hero-banner.webp` (optimized, < 200KB)

---

- [X] T013 [US3] Add background image overlay to hero section in `src/components/HomepageHero/styles.module.css`

**Acceptance Criteria**:
- [ ] Add `::before` pseudo-element to `.heroBanner` with background image
- [ ] Background properties: `background-size: cover`, `background-position: center`
- [ ] Image opacity: 0.3 (allows text to remain readable)
- [ ] Text remains high contrast (white on dark background with image overlay)
- [ ] Image loads in development server

**Implementation Reference** (from plan.md):
```css
/* Modify .heroBanner in styles.module.css */

.heroBanner {
  position: relative;  /* Add */
  padding: 4rem 2rem;
  text-align: center;
  background-color: #0a2342;
  color: #ffffff;
  overflow: hidden;  /* Add */
}

/* Add ::before pseudo-element */
.heroBanner::before {
  content: '';
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background-image: url('/img/hero-banner.webp');
  background-size: cover;
  background-position: center;
  opacity: 0.3;
  z-index: 0;
}

/* Update .heroContainer to ensure text is above image */
.heroContainer {
  position: relative;  /* Add */
  z-index: 1;  /* Add */
  max-width: 1140px;
  margin: 0 auto;
}
```

**File**: `src/components/HomepageHero/styles.module.css`

---

## Phase 5: Polish & Cross-Cutting Concerns

**Goal**: Ensure dark mode compatibility, accessibility compliance, and performance optimization

**Dependencies**: Requires all user stories complete

### Implementation Tasks

- [X] T014 [P] Add dark mode CSS custom properties to `src/css/custom.css`

**Acceptance Criteria**:
- [ ] Define CSS variables for hero background and text colors in `:root`
- [ ] Define dark mode overrides in `[data-theme='dark']`
- [ ] Update hero styles to use CSS variables
- [ ] Test dark mode toggle in browser (click moon icon in navbar)
- [ ] Verify text contrast ≥ 4.5:1 in both light and dark modes

**Implementation**:
```css
/* Add to src/css/custom.css */

:root {
  --hero-bg-color: #0a2342;
  --hero-text-color: #ffffff;
}

[data-theme='dark'] {
  --hero-bg-color: #1a1a2e;
  --hero-text-color: #f0f0f0;
}
```

Then update `.heroBanner` in `styles.module.css`:
```css
.heroBanner {
  /* ... existing styles ... */
  background-color: var(--hero-bg-color, #0a2342);
  color: var(--hero-text-color, #ffffff);
}
```

**Files**:
- `src/css/custom.css`
- `src/components/HomepageHero/styles.module.css` (updated)

---

- [X] T015 Run comprehensive validation tests per `quickstart.md` testing guide

**Acceptance Criteria**:

**Visual Inspection**:
- [ ] Hero displays on homepage only (/, not /intro or other pages)
- [ ] All text readable (title, tagline, description, buttons)
- [ ] Background image visible and not distorted

**Responsive Design** (use DevTools responsive mode):
- [ ] Desktop (1920x1080): Hero fits viewport, title 3rem, buttons horizontal
- [ ] Tablet (768x1024): Title 2rem, padding reduced, buttons horizontal
- [ ] Mobile (375x667): Title 1.5rem, buttons vertical, no horizontal scroll

**Dark Mode**:
- [ ] Toggle dark mode (click moon icon)
- [ ] Hero background/text colors change
- [ ] Text remains readable (contrast ≥ 4.5:1)

**Keyboard Navigation**:
- [ ] Press Tab key → focus moves to "Explore the Book" button
- [ ] Press Tab again → focus moves to "Start Reading" button
- [ ] Press Enter on focused button → navigates correctly
- [ ] Focus outline visible (not removed by CSS)

**Performance** (Lighthouse audit):
- [ ] Open DevTools → Lighthouse tab
- [ ] Run audit for Performance and Accessibility
- [ ] Verify Performance score > 90
- [ ] Verify Accessibility score > 90
- [ ] Verify LCP (Largest Contentful Paint) < 2.5s
- [ ] No console errors or warnings

**Cross-Browser** (manual testing):
- [ ] Chrome/Edge: Hero displays correctly, CTAs work
- [ ] Firefox: Hero displays correctly, CTAs work
- [ ] Safari (if available): Hero displays correctly, WebP or JPEG fallback works

**Build Test**:
- [ ] Run `npm run build` → build completes without errors
- [ ] Run `npm run serve` → production build works
- [ ] Hero displays identically to dev server

**References**: Follow detailed testing steps in [quickstart.md](./quickstart.md)

---

## Dependency Graph

```
Phase 1: Setup
  └── T001 (Verify environment)

Phase 2: US1 (P1) - Basic Hero Display
  ├── T002 [P] (Create component directory)
  ├── T003 (Implement component logic) → depends on T002
  ├── T004 [P] (Create CSS styles)
  ├── T005 (Swizzle DocItem/Layout)
  ├── T006 (Add homepage detection) → depends on T002, T003, T005
  ├── T007 (Test hero display) → depends on T006
  └── T008 (Add responsive styles) → depends on T004

Phase 3: US2 (P2) - CTA Buttons
  ├── T009 (Add CTA buttons to component) → depends on T003
  ├── T010 [P] (Add CTA styles) → depends on T004
  └── T011 (Test CTA navigation) → depends on T009, T010

Phase 4: US3 (P3) - Visual Design
  ├── T012 [P] (Optimize images)
  └── T013 (Add background image) → depends on T012, T004

Phase 5: Polish
  ├── T014 [P] (Dark mode CSS)
  └── T015 (Final validation) → depends on ALL previous tasks

Critical Path: T001 → T002 → T003 → T005 → T006 → T007 → T008 → T009 → T011 → T012 → T013 → T014 → T015
```

---

## Parallel Execution Examples

### After T001 (Setup Complete)

**Can run in parallel**:
- T002 [P] - Create component directory
- T004 [P] - Create CSS styles
- T012 [P] - Optimize images

**Sequential dependency**: T003 must wait for T002

---

### After US1 Complete (T008)

**Can run in parallel**:
- T010 [P] - Add CTA button styles
- T012 [P] - Optimize images (if not done earlier)

**Sequential**: T009 depends on T003 (component must exist)

---

### Before Final Validation (T015)

**Can run in parallel**:
- T014 [P] - Dark mode CSS (independent of other tasks)

---

## MVP Scope (User Story 1 Only)

**Minimum Viable Product** - Basic hero banner on homepage:

**Tasks**: T001-T008 (8 tasks, ~2-3 hours)

**Delivers**:
- Hero section with book title, tagline, description
- Responsive design (desktop, tablet, mobile)
- Homepage-only display (not on other doc pages)

**Not Included in MVP**:
- CTA buttons (US2)
- Background image (US3)
- Dark mode polish (Phase 5)

**MVP Acceptance Test**:
```bash
npm start
# Navigate to http://localhost:3000/physical-ai-and-humanoid-robotics/
# Verify:
# - Hero displays at top of page
# - Title, tagline, description visible and readable
# - Responsive on mobile (375px) and desktop (1920px)
# - Hero does NOT appear on /intro or other doc pages
```

**Decision Point**: After MVP, user can decide to ship minimal version or continue with US2/US3.

---

## Implementation Notes

**File Paths** (all absolute):
- Component: `D:\workspace\nextjs\hackathon-book\src\components\HomepageHero\index.tsx`
- Styles: `D:\workspace\nextjs\hackathon-book\src\components\HomepageHero\styles.module.css`
- Wrapper: `D:\workspace\nextjs\hackathon-book\src\theme\DocItem\Layout\index.tsx`
- Custom CSS: `D:\workspace\nextjs\hackathon-book\src\css\custom.css`
- Images: `D:\workspace\nextjs\hackathon-book\static\img\hero-banner.webp`

**Commands**:
- Dev server: `npm start` (from repo root)
- Build: `npm run build`
- Serve: `npm run serve`
- Swizzle: `npm run swizzle @docusaurus/theme-classic DocItem/Layout -- --wrap`

**References**:
- Plan: [plan.md](./plan.md) - Architecture decisions, implementation patterns
- Research: [research.md](./research.md) - Docusaurus swizzling patterns, responsive design
- Data Model: [data-model.md](./data-model.md) - Component data structures
- Testing: [quickstart.md](./quickstart.md) - Comprehensive testing guide (10-15 minutes)

---

## Task Validation Summary

**Format Compliance**:
- ✅ All tasks use checkbox format: `- [ ]`
- ✅ All tasks have sequential IDs: T001-T015
- ✅ Parallelizable tasks marked with [P]
- ✅ User story tasks labeled: [US1], [US2], [US3]
- ✅ All tasks include file paths

**Completeness**:
- ✅ All 3 user stories covered (US1 P1, US2 P2, US3 P3)
- ✅ Each user story has independent test criteria
- ✅ Setup phase included (T001)
- ✅ Polish phase included (T014-T015)
- ✅ Dependency graph shows execution order
- ✅ MVP scope clearly defined (US1 only)

**User Story Coverage**:
- US1 (P1): 8 tasks - Basic hero display ✅
- US2 (P2): 3 tasks - CTA buttons ✅
- US3 (P3): 2 tasks - Visual design ✅
- Polish: 2 tasks - Dark mode, validation ✅

**Total**: 15 tasks, ~6-8 hours estimated for full implementation

---

**Ready for Implementation**: All tasks are specific, testable, and include exact file paths for immediate execution.

**Next Step**: Start with T001 (verify environment) or proceed directly to MVP scope (T001-T008).
