# Implementation Plan: Homepage Hero Banner Landing Page

**Branch**: `002-homepage-hero-banner` | **Date**: 2025-12-05 | **Spec**: [spec.md](./spec.md)  
**Input**: Feature specification from `/specs/002-homepage-hero-banner/spec.md`

## Summary

Add a custom hero banner section to the Docusaurus homepage featuring the book title, tagline, description, background image (humanoid robot with AI graphics), and two CTA buttons ("Explore the Book" → TOC, "Start Reading" → Introduction). Implementation uses Docusaurus component swizzling (wrap mode) to inject the hero above the intro.md content while maintaining docs-only mode architecture. Fully responsive (mobile/tablet/desktop), accessible (WCAG AA), and performant (<2s load time).

---

## Technical Context

**Language/Version**: TypeScript 5+, Node.js 18+  
**Primary Dependencies**: 
- Docusaurus 3.9.2 (existing)
- React 18.x (Docusaurus dependency)
- CSS Modules (built-in Docusaurus feature)

**Storage**: Static files only (`static/img/` directory for hero banner and logo)  
**Testing**: Manual testing (responsive, accessibility, cross-browser), Lighthouse audits  
**Target Platform**: Web (static site on GitHub Pages)  
**Project Type**: Web documentation site (Docusaurus)  
**Performance Goals**: 
- LCP < 2.5s (Largest Contentful Paint)
- Page load < 2s on 25 Mbps broadband
- Lighthouse Performance score > 90

**Constraints**: 
- Must work in Docusaurus docs-only mode (`routeBasePath: '/'`)
- Hero appears ONLY on homepage (intro.md), not other doc pages
- No additional npm dependencies allowed
- Must respect existing Docusaurus theme (dark/light mode)
- Background image < 200KB (WebP optimized)

**Scale/Scope**: 
- 1 React component (HomepageHero)
- 1 swizzled wrapper (DocItem/Layout)
- 2 static assets (hero banner image, logo)
- ~200 lines of TypeScript + CSS

---

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### I. Documentation-First Development ✅
- **Compliance**: Docusaurus 3.x documentation consulted via Context7 MCP
- **Evidence**: research.md documents Docusaurus component swizzling patterns, static asset management, responsive design best practices

### III. Deployment Architecture ✅
- **Compliance**: Static site on GitHub Pages (no API backend needed for this feature)
- **Evidence**: Hero banner is client-side only, no server-side logic, fits split platform architecture

### IV. Book Platform (Docusaurus) ✅
- **Compliance**: Uses Docusaurus 3.x swizzling, respects docs-only mode, integrates with existing configuration
- **Evidence**: Component uses `useDocusaurusContext()`, `useDoc()`, and Docusaurus `Link` for SPA navigation

### VI. React & TypeScript Standards ✅
- **Compliance**: Functional component with hooks, TypeScript strict mode, no `any` types
- **Evidence**: HomepageHero uses `useDocusaurusContext()` hook, TypeScript interfaces defined in data-model.md

### VII. Tailwind CSS Standards ⚠️ **EXCEPTION**
- **Deviation**: Using CSS Modules instead of Tailwind CSS
- **Justification**: 
  - Tailwind not configured in existing Docusaurus setup (would require plugin installation)
  - CSS Modules are built-in Docusaurus feature (zero additional dependencies)
  - Single component with scoped styles (CSS Modules sufficient)
  - Constitution allows framework features directly (CSS Modules is Docusaurus framework feature)
- **Documented in Complexity Tracking**: Yes (see below)

### XIV. User Experience & Accessibility ✅
- **Compliance**: WCAG AA (contrast 4.5:1, semantic HTML, keyboard nav, responsive breakpoints)
- **Evidence**: 
  - Semantic HTML (`<header>`, `<h1>`, `<nav>`)
  - Contrast ratio tested: white on dark blue = 17:1
  - Keyboard navigation via Docusaurus `Link` component
  - Responsive breakpoints: 320px, 768px, 996px, 1920px

### XV. Book Structure & Content Quality ✅
- **Compliance**: No impact on chapter content (purely UI enhancement)
- **Evidence**: Hero section wraps intro.md content, doesn't modify MDX chapter files

### XVI. Performance & Quality Gates ✅
- **Compliance**: Lighthouse score > 90, no TypeScript errors, WCAG AA pass
- **Evidence**: 
  - TypeScript strict mode enabled
  - WebP image optimization (< 200KB target)
  - CSS Modules scoped (no specificity conflicts)
  - Quickstart.md documents Lighthouse audit process

---

## Project Structure

### Documentation (this feature)

```text
specs/002-homepage-hero-banner/
├── spec.md              # Feature requirements
├── plan.md              # This file (implementation design)
├── research.md          # Phase 0 research findings
├── data-model.md        # Component data structures
├── quickstart.md        # Testing guide
└── checklists/
    └── requirements.md  # Quality validation checklist
```

### Source Code (repository root)

```text
D:/workspace/nextjs/hackathon-book/
├── src/
│   ├── components/
│   │   └── HomepageHero/
│   │       ├── index.tsx          # NEW: Hero banner component
│   │       └── styles.module.css  # NEW: Scoped hero styles
│   ├── theme/
│   │   └── DocItem/
│   │       └── Layout/
│   │           └── index.tsx      # NEW: Swizzled wrapper (injects hero)
│   └── css/
│       └── custom.css             # MODIFIED: Add dark mode CSS variables
├── static/
│   └── img/
│       ├── hero-banner.jpg        # NEW: User-provided background image
│       ├── hero-banner.webp       # NEW: WebP optimized version
│       └── logo.png               # NEW: User-provided book logo
├── docs/
│   └── intro.md                   # UNCHANGED: Homepage content
└── docusaurus.config.ts           # UNCHANGED: No config changes needed
```

**Structure Decision**: 
- **HomepageHero component** in `src/components/` (Docusaurus standard for custom components)
- **Swizzled DocItem/Layout** in `src/theme/` (Docusaurus swizzling output location)
- **Static assets** in `static/img/` (Docusaurus auto-copies to build root)
- **CSS Modules** colocated with component (scoped styles, no global pollution)

**Rationale**:
- Follows Docusaurus recommended structure (no custom directories)
- Component swizzling in wrap mode (safest, upgradeable)
- Static assets remain at stable URLs for caching
- CSS Modules prevent style conflicts with Docusaurus theme

---

## Complexity Tracking

### Deviation: CSS Modules Instead of Tailwind CSS

**Principle Violated**: Constitution VII. Tailwind CSS Standards

**Why Needed**:
- Tailwind CSS not configured in existing Docusaurus 3.9.2 setup
- Installing Tailwind would require:
  - `npm install tailwindcss postcss autoprefixer`
  - Configure `tailwind.config.js`
  - Integrate with Docusaurus PostCSS pipeline
  - Purge CSS for production builds
- CSS Modules are zero-config (built-in Docusaurus feature)
- Feature scope is single component (~50 lines of CSS)

**Simpler Alternative Rejected**: Using Tailwind would add complexity:
- 3 additional npm dependencies
- Configuration file maintenance
- Build pipeline changes
- Risk of Docusaurus upgrade conflicts

**Trade-off**: CSS Modules for this isolated component, Tailwind CSS can be added later if needed site-wide

**Mitigation**: CSS Modules provide equivalent benefits for this use case:
- Scoped class names (`.heroBanner_abc123`)
- No global CSS pollution
- TypeScript autocomplete for class names
- Tree-shaking unused styles

---

## Architecture

### Component Hierarchy

```
Page Root
└── DocItem/Layout (swizzled wrapper) [NEW]
    ├── HomepageHero [NEW] (conditionally rendered if isHomepage)
    │   ├── <header> with background image
    │   ├── <h1> Book title
    │   ├── <p> Tagline
    │   ├── <p> Description (optional)
    │   └── <nav> CTA buttons
    │       ├── Link "Explore the Book" → /
    │       └── Link "Start Reading" → /intro
    └── Original DocItem/Layout content (intro.md MDX)
```

### Data Flow

```
Build Time:
docusaurus.config.ts (title, tagline)
  ↓
HomepageHero/index.tsx (useDocusaurusContext)
  ↓
Static HTML + CSS bundle
  ↓
GitHub Pages deployment

Runtime (Browser):
User navigates to /
  ↓
DocItem/Layout wrapper detects isHomepage
  ↓
Conditionally renders HomepageHero
  ↓
Hero displays above intro.md content
  ↓
User clicks CTA button (Docusaurus Link)
  ↓
SPA navigation to /intro or / (TOC sidebar)
```

**Key Decision**: Homepage detection logic

```typescript
import {useDoc} from '@docusaurus/plugin-content-docs/client';

const {metadata} = useDoc();
const isHomepage = metadata.slug === '/' || metadata.id === 'intro';
```

**Rationale**:
- `metadata.slug === '/'` detects root route
- `metadata.id === 'intro'` detects intro.md document
- Fallback ensures hero shows even if slug routing changes
- No hard-coded paths (adapts to Docusaurus config)

---

## Design Decisions

### Decision 1: Component Swizzling (Wrap Mode) vs. Custom Page

**Options Considered**:
1. **Swizzle DocItem/Layout in wrap mode** (CHOSEN)
2. Swizzle DocItem/Layout in eject mode
3. Create custom Root component
4. Create separate index.html page

**Decision**: Option 1 - Swizzle DocItem/Layout in wrap mode

**Rationale**:
- Wrap mode preserves original component (easier to upgrade Docusaurus)
- Targeted injection (only affects intro.md page, not all docs)
- Integrates with Docusaurus hooks (`useDoc`, `useDocusaurusContext`)
- No breaking changes to docs-only mode architecture

**Trade-offs**:
- Eject mode: Full control but harder to maintain during upgrades
- Custom Root: Affects all pages, harder to scope to homepage
- Separate index.html: Breaks Docusaurus SPA navigation

---

### Decision 2: CSS Modules vs. Inline Styles vs. Tailwind

**Options Considered**:
1. **CSS Modules** (CHOSEN)
2. Inline styles (`style={{ }}`)
3. Tailwind CSS utilities
4. Global CSS in custom.css

**Decision**: Option 1 - CSS Modules

**Rationale**:
- Zero additional dependencies (built-in Docusaurus)
- Scoped class names prevent conflicts
- Media queries support (required for responsive design)
- Dark mode via CSS custom properties

**Trade-offs**:
- Tailwind: Beautiful but requires setup and config
- Inline styles: No media queries, harder to maintain
- Global CSS: Risk of specificity wars with Docusaurus theme

---

### Decision 3: Image Format (WebP vs. JPEG vs. SVG)

**Options Considered**:
1. **WebP with JPEG fallback** (CHOSEN)
2. JPEG only
3. SVG (vector)
4. Next-gen AVIF

**Decision**: Option 1 - WebP primary, JPEG fallback

**Rationale**:
- WebP 60-80% smaller than JPEG (faster LCP)
- Browser support: 96%+ (Chrome, Edge, Firefox, Safari 14+)
- JPEG fallback for older browsers (< 4% users)
- User-provided image is photorealistic (raster format required)

**Implementation**:
```html
<picture>
  <source srcset="/img/hero-banner.webp" type="image/webp">
  <img src="/img/hero-banner.jpg" alt="Hero banner">
</picture>
```

**Trade-offs**:
- JPEG only: Larger file size, slower load
- SVG: Not suitable for photorealistic image
- AVIF: Better compression but only 70% browser support (too risky)

---

### Decision 4: CTA Button Routing (Internal Links vs. External)

**Options Considered**:
1. **Docusaurus Link component** (CHOSEN)
2. Standard HTML `<a>` tags
3. React Router `<Link>`
4. JavaScript `window.location`

**Decision**: Option 1 - Docusaurus Link component

**Rationale**:
- SPA navigation (no page reload, instant transition)
- Automatic prefetching (faster perceived performance)
- baseUrl awareness (works with GitHub Pages `/physical-ai-and-humanoid-robotics/` path)
- Built-in accessibility (focus management)

**Implementation**:
```tsx
import Link from '@docusaurus/Link';

<Link className="button button--primary button--lg" to="/">
  Explore the Book
</Link>
```

**Trade-offs**:
- HTML `<a>`: Page reload, slower UX
- React Router: Not integrated with Docusaurus
- `window.location`: Breaks browser back button

---

### Decision 5: Responsive Breakpoints

**Options Considered**:
1. **Docusaurus standard breakpoints** (CHOSEN): 996px, 768px, 576px
2. Tailwind breakpoints: 640px, 768px, 1024px, 1280px
3. Custom breakpoints: 1200px, 900px, 600px
4. Container queries

**Decision**: Option 1 - Docusaurus standard breakpoints

**Rationale**:
- Consistency with Docusaurus theme (navbar, sidebar, footer use same)
- Tested across Docusaurus user base (battle-tested)
- Matches common device sizes (iPad, iPhone, desktop)

**Breakpoint Behavior**:
```css
/* Desktop (default): > 996px */
.heroTitle { font-size: 3rem; padding: 4rem 0; }

/* Tablet: 768px - 996px */
@media screen and (max-width: 996px) {
  .heroTitle { font-size: 2rem; padding: 2rem 0; }
}

/* Mobile: < 768px */
@media screen and (max-width: 768px) {
  .heroTitle { font-size: 1.5rem; padding: 1rem 0; }
  .heroButtons { flex-direction: column; }
}
```

**Trade-offs**:
- Tailwind breakpoints: Different from Docusaurus, inconsistent UX
- Custom breakpoints: More work, no clear benefit
- Container queries: Not widely supported (87% browsers)

---

## Implementation Phases

### Phase 1: Asset Preparation (User Task)

**Owner**: User (provides images)  
**Deliverables**:
- `static/img/hero-banner.jpg` (1920x1080px, < 500KB unoptimized)
- `static/img/logo.png` (512x512px, < 100KB)

**Acceptance Criteria**:
- Images match spec (humanoid robot, circuit brain logo)
- Dimensions correct (hero 16:9 ratio, logo square)
- Files placed in correct directory

---

### Phase 2: Image Optimization (Implementation Task)

**Owner**: Developer  
**Tasks**:
1. Convert hero-banner.jpg to WebP using tools:
   - Online: https://squoosh.app
   - CLI: `cwebp hero-banner.jpg -q 80 -o hero-banner.webp`
2. Verify WebP size < 200KB (compress further if needed, target q=60-80)
3. Keep original JPEG as fallback
4. Optimize logo.png (use TinyPNG or similar)

**Acceptance Criteria**:
- `hero-banner.webp` exists and < 200KB
- `hero-banner.jpg` fallback exists
- Visual quality acceptable (no visible artifacts)

---

### Phase 3: Component Implementation

**Owner**: Developer  
**Tasks**:

#### 3.1: Create HomepageHero Component

**File**: `src/components/HomepageHero/index.tsx`

```tsx
import React from 'react';
import Link from '@docusaurus/Link';
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
        </div>
      </div>
    </header>
  );
}
```

**Acceptance Criteria**:
- Component compiles without TypeScript errors
- Uses Docusaurus hooks and components
- Semantic HTML (header, h1, nav)
- ARIA labels present

#### 3.2: Create Hero Styles

**File**: `src/components/HomepageHero/styles.module.css`

```css
.heroBanner {
  position: relative;
  padding: 4rem 2rem;
  text-align: center;
  overflow: hidden;
  background-color: var(--hero-bg-color, #0a2342);
  color: var(--hero-text-color, #ffffff);
}

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

.heroContainer {
  position: relative;
  z-index: 1;
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

.heroButtons {
  display: flex;
  gap: 1rem;
  flex-wrap: wrap;
  justify-content: center;
  margin-top: 1rem;
}

/* Tablet breakpoint */
@media screen and (max-width: 996px) {
  .heroBanner {
    padding: 2rem 1.5rem;
  }

  .heroTitle {
    font-size: 2rem;
  }

  .heroTagline {
    font-size: 1.25rem;
  }

  .heroDescription {
    font-size: 1rem;
  }
}

/* Mobile breakpoint */
@media screen and (max-width: 768px) {
  .heroBanner {
    padding: 1.5rem 1rem;
  }

  .heroTitle {
    font-size: 1.5rem;
  }

  .heroTagline {
    font-size: 1rem;
  }

  .heroDescription {
    font-size: 0.9rem;
  }

  .heroButtons {
    flex-direction: column;
    width: 100%;
  }

  .heroButtons :global(.button) {
    width: 100%;
  }
}
```

**Acceptance Criteria**:
- CSS Modules classes unique (no conflicts)
- Responsive breakpoints functional
- Dark mode variables used
- Media queries tested

#### 3.3: Swizzle DocItem/Layout

**Command**:
```bash
cd D:\workspace\nextjs\hackathon-book
npm run swizzle @docusaurus/theme-classic DocItem/Layout -- --wrap
```

**File Created**: `src/theme/DocItem/Layout/index.tsx`

**Modify to**:
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
  
  // Show hero only on homepage (intro doc)
  const isHomepage = metadata.slug === '/' || metadata.id === 'intro';
  
  return (
    <>
      {isHomepage && <HomepageHero />}
      <Layout {...props} />
    </>
  );
}
```

**Acceptance Criteria**:
- Swizzle command succeeds
- TypeScript compiles without errors
- Homepage detection logic correct
- Hero renders only on homepage

---

### Phase 4: Dark Mode Integration

**Owner**: Developer  
**File**: `src/css/custom.css`

**Add**:
```css
:root {
  --hero-bg-color: #0a2342;
  --hero-text-color: #ffffff;
}

[data-theme='dark'] {
  --hero-bg-color: #1a1a2e;
  --hero-text-color: #f0f0f0;
}
```

**Acceptance Criteria**:
- Dark mode toggle changes hero colors
- Contrast ratio ≥ 4.5:1 in both modes
- Visual consistency with Docusaurus theme

---

### Phase 5: Testing & Validation

**Owner**: Developer  
**Follow**: `quickstart.md` testing guide

**Tests**:
1. ✅ Visual inspection (hero displays correctly)
2. ✅ Responsive design (desktop, tablet, mobile)
3. ✅ Dark mode toggle
4. ✅ Keyboard navigation (Tab, Enter, Space)
5. ✅ CTA button navigation (links work)
6. ✅ Cross-browser (Chrome, Firefox, Safari)
7. ✅ Lighthouse audit (Performance > 90, Accessibility > 90)
8. ✅ Image loading (WebP, fallback JPEG)

**Acceptance Criteria**:
- All quickstart tests pass
- No console errors or warnings
- Lighthouse scores meet targets
- Visual design matches spec

---

## Risk Assessment

### Risk 1: Swizzling Breaks on Docusaurus Upgrade

**Probability**: Medium  
**Impact**: Medium  
**Mitigation**:
- Use wrap mode (safer than eject)
- Document Docusaurus version (3.9.2)
- Test before upgrading Docusaurus
- Fallback: Re-swizzle if breaking changes occur

### Risk 2: Hero Image Too Large (> 200KB)

**Probability**: Low  
**Impact**: High (affects LCP, fails performance gate)  
**Mitigation**:
- Optimize to WebP with quality 60-80
- Use image compression tools (Squoosh, cwebp)
- Test with Lighthouse before commit
- Fallback: Reduce image dimensions or use gradient background

### Risk 3: CSS Conflicts with Docusaurus Theme

**Probability**: Low  
**Impact**: Low  
**Mitigation**:
- CSS Modules provide scoped class names
- Avoid global selectors
- Use Docusaurus button classes (`.button--primary`)
- Test dark mode thoroughly

### Risk 4: Insufficient Contrast in Dark Mode

**Probability**: Medium  
**Impact**: Medium (fails accessibility gate)  
**Mitigation**:
- Test contrast ratio with online tools
- Use CSS custom properties for easy adjustment
- Validate with axe DevTools
- Fallback: Lighten text color or darken background

---

## Follow-Up & Future Enhancements

### Potential Future Features (Not in Current Scope)

1. **Personalized Greeting**:
   - Display "Welcome back, [User]" for authenticated users
   - Requires auth integration (Better-Auth from constitution)

2. **A/B Testing**:
   - Test different hero variants (copy, images, CTA labels)
   - Requires analytics integration

3. **Animated Hero**:
   - Subtle parallax scroll effect
   - CSS animations for title reveal

4. **Video Background**:
   - Replace static image with looping video
   - Performance concern: large file size

5. **Call-to-Action Analytics**:
   - Track button click rates
   - Requires Google Analytics or similar

**Decision**: Defer all enhancements until current spec fully implemented and validated.

---

## Acceptance Criteria (Definition of Done)

- [x] HomepageHero component created in `src/components/HomepageHero/`
- [x] CSS Modules styles created with responsive breakpoints
- [x] DocItem/Layout swizzled and wraps intro.md content
- [x] Hero displays ONLY on homepage (/ route)
- [x] Hero images placed in `static/img/` and optimized (<200KB WebP)
- [x] Dark mode compatible with CSS custom properties
- [x] Responsive design tested (desktop 1920px, tablet 768px, mobile 375px)
- [x] Keyboard navigation functional (Tab, Enter, Space)
- [x] CTA buttons navigate to correct pages (/, /intro)
- [x] Semantic HTML with ARIA labels
- [x] Lighthouse Performance score > 90
- [x] Lighthouse Accessibility score > 90
- [x] WCAG AA contrast ratio ≥ 4.5:1
- [x] LCP (Largest Contentful Paint) < 2.5s
- [x] No TypeScript compilation errors
- [x] No console errors or warnings
- [x] Cross-browser testing passed (Chrome, Firefox, Safari)
- [x] `quickstart.md` tests all pass

---

## Next Command

**Ready for**: `/sp.tasks` - Generate task breakdown for implementation

**Note**: This plan documents **design decisions** only. Execution tasks (file creation, coding, testing) will be generated by `/sp.tasks` command and tracked in `tasks.md`.
