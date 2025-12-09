# Research: Homepage Hero Banner Implementation

**Feature**: 002-homepage-hero-banner  
**Date**: 2025-12-05  
**Research Phase**: Phase 0

## Research Questions Resolved

### 1. How to customize Docusaurus homepage in docs-only mode?

**Decision**: Use swizzled DocItem/Layout component wrapper to inject hero section above content

**Rationale**:
- Project uses `routeBasePath: '/'` making docs the homepage
- Swizzling in wrap mode is safest (doesn't eject entire component, easier to maintain)
- Allows conditional rendering (hero only on homepage, not all doc pages)
- Preserves all existing Docusaurus functionality

**Alternatives Considered**:
- Custom Root component: Works but less targeted, affects all pages
- Ejecting entire DocItem component: Too invasive, harder to upgrade Docusaurus
- Creating separate index.html: Breaks docs-only mode architecture

**Implementation Pattern**:
```bash
npm run swizzle @docusaurus/theme-classic DocItem/Layout -- --wrap
```

Then wrap with homepage detection:
```tsx
const {metadata} = useDoc();
const isHomepage = metadata.slug === '/' || metadata.id === 'intro';
```

---

### 2. Where to place custom React components in Docusaurus?

**Decision**: `src/components/HomepageHero/` with CSS modules for styling

**Rationale**:
- Follows Docusaurus recommended structure (`src/components/`)
- CSS Modules provide scoped styling (prevents global CSS conflicts)
- Colocated component logic and styles for maintainability
- Aligns with constitution principle of avoiding custom CSS frameworks

**File Structure**:
```
src/
├── components/
│   └── HomepageHero/
│       ├── index.tsx
│       └── styles.module.css
└── theme/
    └── DocItem/
        └── Layout/
            └── index.tsx (swizzled wrapper)
```

**Alternatives Considered**:
- Global CSS in custom.css: Harder to scope, increases specificity conflicts
- Inline styles: Not performant, harder to maintain, no media queries
- Tailwind utilities: Considered but CSS Modules simpler for single component

---

### 3. How to integrate hero banner background image and logo?

**Decision**: Place assets in `static/img/` and reference via CSS url() or React require()

**Rationale**:
- `static/` directory gets copied to build root automatically
- Assets remain at stable URLs for caching
- CSS `url('/img/hero-banner.jpg')` works in both dev and production
- Aligns with Docusaurus asset management best practices

**Asset Placement**:
```
static/
└── img/
    ├── hero-banner.jpg (1920x1080px, user-provided)
    ├── logo.png (circular badge, user-provided)
    └── favicon.ico (existing)
```

**Reference Methods**:
- **CSS** (for background): `background-image: url('/img/hero-banner.jpg')`
- **React** (for logo): `<img src={require('@site/static/img/logo.png').default} />`

**Alternatives Considered**:
- Importing images in JSX: Works but unnecessary for background images
- Using CDN URLs: Adds external dependency, not needed for static assets
- Base64 encoding: Increases bundle size, bad for large images

---

### 4. Responsive design approach for hero section?

**Decision**: CSS media queries with mobile-first breakpoints matching Docusaurus standards

**Rationale**:
- Docusaurus uses standard breakpoints: 996px (desktop), 768px (tablet), 576px (mobile)
- CSS Modules support media queries natively
- Aligns with constitution mobile-first principle
- No JavaScript needed for responsive behavior

**Breakpoint Strategy**:
```css
/* Desktop (default) */
.heroTitle { font-size: 3rem; padding: 4rem 0; }

/* Tablet */
@media screen and (max-width: 996px) {
  .heroTitle { font-size: 2rem; padding: 2rem 0; }
}

/* Mobile */
@media screen and (max-width: 768px) {
  .heroTitle { font-size: 1.5rem; padding: 1rem 0; }
}
```

**Responsive Elements**:
- Hero title: 3rem → 2rem → 1.5rem
- Padding: 4rem → 2rem → 1rem
- CTA buttons: horizontal flex → vertical stack on mobile
- Background image: cover with center position (no horizontal scroll)

**Alternatives Considered**:
- JavaScript-based responsive: Overkill, CSS is sufficient
- Container queries: Not widely supported yet, media queries safer
- Tailwind responsive utilities: Not using Tailwind for this feature (CSS Modules simpler)

---

### 5. Dark mode compatibility?

**Decision**: Use CSS custom properties that respect Docusaurus `[data-theme]` attribute

**Rationale**:
- Docusaurus built-in dark mode uses `[data-theme='dark']` attribute
- CSS custom properties automatically inherit theme
- No JavaScript needed for theme switching
- Consistent with Docusaurus design system

**Implementation Pattern**:
```css
/* Light mode (default) */
:root {
  --hero-bg-color: #0a2342;
  --hero-text-color: #ffffff;
}

/* Dark mode */
[data-theme='dark'] {
  --hero-bg-color: #1a1a2e;
  --hero-text-color: #f0f0f0;
}

.heroBanner {
  background-color: var(--hero-bg-color);
  color: var(--hero-text-color);
}
```

**Alternatives Considered**:
- Hardcoded colors: Breaks dark mode, poor UX
- JavaScript theme detection: Unnecessary, CSS handles it
- Duplicate stylesheets: Harder to maintain

---

### 6. CTA button destinations?

**Decision** (from spec clarifications):
- Primary CTA: "Explore the Book" → `/` (Table of Contents, same as Docusaurus sidebar)
- Secondary CTA: "Start Reading" → `/intro` (Introduction/Chapter 1)

**Rationale**:
- Clarified in spec session 2025-12-05
- Dual-path UX: browse (TOC) vs dive in (start reading)
- Uses Docusaurus `Link` component for internal routing (SPA navigation)

**Implementation**:
```tsx
import Link from '@docusaurus/Link';

<Link className="button button--primary button--lg" to="/">
  Explore the Book
</Link>
<Link className="button button--secondary button--lg" to="/intro">
  Start Reading
</Link>
```

**Note**: Docusaurus `Link` provides:
- Automatic prefetching
- Active state styling
- SPA navigation without page reload
- baseUrl awareness for GitHub Pages deployment

---

### 7. Accessibility requirements?

**Decision**: Semantic HTML with ARIA labels, keyboard navigation, sufficient contrast

**Rationale**:
- Constitution requires WCAG AA compliance (FR-008, FR-009, FR-010 in spec)
- Semantic HTML (`<header>`, `<h1>`) aids screen readers
- Contrast ratio 4.5:1 for body text, 3:1 for large text
- Keyboard navigation via native `<a>` elements

**Implementation Checklist**:
- ✅ Use `<header>` for hero section (semantic landmark)
- ✅ Use `<h1>` for book title (proper heading hierarchy)
- ✅ Alt text for logo image: "Physical AI & Humanoid Robotics logo"
- ✅ Sufficient contrast: white text (#ffffff) on dark blue (#0a2342) = 17:1 ratio
- ✅ Focus indicators on CTA buttons (Docusaurus provides via `.button` class)
- ✅ Keyboard accessible (Docusaurus `Link` component handles Tab/Enter)

**Validation Tools**:
- axe DevTools for automated accessibility audit
- Lighthouse accessibility score > 90
- Manual keyboard navigation testing (Tab, Enter, Space)

---

### 8. Performance optimization?

**Decision**: Optimize images (WebP format, lazy loading), CSS-only animations

**Rationale**:
- Constitution requires < 2s load time (SC-004 in spec)
- Hero banner is above-the-fold (no lazy loading for critical content)
- Background image should be optimized for web (target < 200KB)
- CSS animations perform better than JavaScript

**Optimization Strategy**:
- Convert hero-banner.jpg to WebP (50-80% smaller than JPEG)
- Provide multiple resolutions: hero-banner.webp (1920px), hero-banner@2x.webp (3840px for Retina)
- Use `<picture>` element for WebP fallback:
  ```html
  <picture>
    <source srcset="/img/hero-banner.webp" type="image/webp">
    <img src="/img/hero-banner.jpg" alt="Hero">
  </picture>
  ```
- Lazy load logo if not critical (but likely visible above fold)

**Performance Targets** (from spec):
- LCP: < 2.5s (hero image is LCP element)
- FID: < 100ms (minimal JavaScript, mostly static)
- CLS: < 0.1 (fixed dimensions for hero section)

**Alternatives Considered**:
- Inline SVG for hero: File too large, not suitable
- JavaScript lazy loading library: Overkill for single component
- Video background: Beautiful but massive performance hit

---

## Technology Stack Confirmed

**Frontend Framework**: Docusaurus 3.9.2 (existing)  
**Component Library**: React 18.x (Docusaurus dependency)  
**Styling**: CSS Modules (scoped styles)  
**Routing**: Docusaurus Link (internal SPA navigation)  
**Assets**: Static files in `/static/img/`  
**Build**: Docusaurus build pipeline (existing)  
**Deployment**: GitHub Pages via GitHub Actions (existing)

**No additional dependencies required** - all functionality available in existing Docusaurus setup.

---

## Implementation Risks & Mitigations

### Risk 1: Swizzling breaks on Docusaurus upgrade
**Mitigation**: Use wrap mode (safer than eject), document version (3.9.2), test before upgrading

### Risk 2: Hero image file size affects LCP
**Mitigation**: Optimize to WebP < 200KB, serve responsive sizes, use CDN caching (GitHub Pages)

### Risk 3: CSS specificity conflicts with Docusaurus theme
**Mitigation**: CSS Modules provide scoped class names (`.heroBanner_abc123`), avoid global selectors

### Risk 4: Dark mode colors insufficient contrast
**Mitigation**: Test with color contrast analyzer, use CSS custom properties for easy adjustment

---

## Next Steps (Phase 1)

1. Create data model (minimal - just hero content structure)
2. Design component architecture (HomepageHero component)
3. Define file structure (src/components/, static/img/)
4. Document quickstart guide for testing

**Phase 0 Complete** ✅
