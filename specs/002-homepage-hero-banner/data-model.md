# Data Model: Homepage Hero Banner

**Feature**: 002-homepage-hero-banner  
**Date**: 2025-12-05  
**Phase**: 1

## Overview

The homepage hero banner is a **static UI component** with no dynamic data or database entities. All content is hardcoded or configured in the Docusaurus configuration file.

## Entities

### HeroContent (Static Configuration)

**Type**: TypeScript interface for component props  
**Storage**: None (hardcoded in component)  
**Lifecycle**: Build-time only (static site generation)

```typescript
interface HeroContent {
  title: string;          // Book title from docusaurus.config.ts
  tagline: string;        // Subtitle from docusaurus.config.ts
  description?: string;   // Optional longer description (50-150 words)
  backgroundImage: string; // Path to hero banner image
  logo?: string;          // Path to book logo
  primaryCTA: CTAButton;  // "Explore the Book" button
  secondaryCTA: CTAButton; // "Start Reading" button
}

interface CTAButton {
  label: string;  // Button text
  href: string;   // Internal Docusaurus route or external URL
  variant: 'primary' | 'secondary'; // Button style
}
```

**Example Data**:
```typescript
const heroContent: HeroContent = {
  title: "Physical AI & Humanoid Robotics",
  tagline: "A comprehensive guide to building intelligent robots",
  description: "Master ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action systems. From simulation to real hardware deployment.",
  backgroundImage: "/img/hero-banner.jpg",
  logo: "/img/logo.png",
  primaryCTA: {
    label: "Explore the Book",
    href: "/",
    variant: "primary"
  },
  secondaryCTA: {
    label: "Start Reading",
    href: "/intro",
    variant: "secondary"
  }
};
```

---

## Static Assets

### Asset 1: Hero Background Image

**File**: `static/img/hero-banner.jpg` (or .webp)  
**Dimensions**: 1920x1080px (16:9 ratio)  
**Format**: WebP (primary), JPEG (fallback)  
**Size Target**: < 200KB (optimized for web)  
**Source**: User-provided (photorealistic humanoid robot with AI/brain graphics)

**Responsive Variants**:
- `hero-banner.webp` - 1920x1080px (desktop)
- `hero-banner@2x.webp` - 3840x2160px (Retina displays)
- `hero-banner.jpg` - 1920x1080px (fallback for browsers without WebP)

**Usage**:
```css
.heroBanner {
  background-image: url('/img/hero-banner.jpg');
  background-size: cover;
  background-position: center;
}
```

### Asset 2: Book Logo

**File**: `static/img/logo.png`  
**Dimensions**: 512x512px (square, 1:1 ratio)  
**Format**: PNG with transparency  
**Size Target**: < 50KB  
**Source**: User-provided (circular badge with circuit brain pattern)

**Variants**:
- `logo.png` - 512x512px (primary)
- `logo.svg` - Vector version (if available, preferred for scalability)

**Usage**:
```tsx
<img 
  src={require('@site/static/img/logo.png').default} 
  alt="Physical AI & Humanoid Robotics logo"
  width={80}
  height={80}
/>
```

**Note**: Logo is optional in hero section (spec doesn't require it above fold, but available for navbar/footer)

---

## Component State

### HomepageHero Component State

**State Management**: None required (fully static component)

**No useState/useReducer** - All data is props or configuration constants

**Rationale**:
- Hero section has no user interactions beyond CTA clicks
- CTA clicks use Docusaurus Link (handled by router)
- No form inputs, toggles, or dynamic content
- Dark mode handled by CSS custom properties (no JS state)

**Edge Case**: If personalization added in future (e.g., "Welcome back, [User]"), would require:
```typescript
const {user} = useAuth(); // Future enhancement
const greeting = user ? `Welcome back, ${user.name}` : tagline;
```

But current spec has no personalization requirement, so state remains static.

---

## Data Flow

### Build Time (Static Site Generation)

```
docusaurus.config.ts 
  ↓ (title, tagline)
HomepageHero/index.tsx 
  ↓ (import useDocusaurusContext)
Static HTML/CSS Bundle
  ↓ (deployed to GitHub Pages)
Visitor Browser
```

**No Runtime Data Fetching**:
- No API calls
- No database queries
- No user-specific content
- No localStorage/sessionStorage

All content is baked into the HTML at build time.

---

## Validation Rules

### Content Constraints (from spec FR-002, FR-003, FR-004)

```typescript
// Type-safe content validation
const HERO_CONTENT_CONSTRAINTS = {
  title: {
    minLength: 10,
    maxLength: 60,
    required: true,
    example: "Physical AI & Humanoid Robotics"
  },
  tagline: {
    minLength: 20,
    maxLength: 100,
    required: true,
    example: "A comprehensive guide to building intelligent robots"
  },
  description: {
    minLength: 50,
    maxLength: 150, // Words from FR-004
    required: false,
    example: "Master ROS 2, Gazebo, NVIDIA Isaac..."
  }
} as const;
```

**Validation Timing**: Build-time TypeScript type checking (no runtime validation needed)

**Error Handling**: TypeScript compiler errors if constraints violated

---

## Relationships

**No database relationships** - this is a static component.

**Docusaurus Integration Points**:
- `useDocusaurusContext()` → reads site config (title, tagline)
- `useDoc()` → detects if current page is homepage (metadata.slug === '/')
- `Link` component → handles internal routing for CTAs

**Dependency Graph**:
```
DocItem/Layout (swizzled wrapper)
  ↓ depends on
HomepageHero Component
  ↓ depends on
  - useDocusaurusContext (Docusaurus hook)
  - Link (Docusaurus component)
  - styles.module.css (CSS Modules)
  - /img/hero-banner.jpg (static asset)
```

---

## Accessibility Data Attributes

### ARIA Labels and Semantic HTML

```tsx
<header 
  className={styles.heroBanner}
  role="banner"        // Explicit landmark for screen readers
  aria-label="Homepage hero section"
>
  <h1 className={styles.heroTitle}>
    {siteConfig.title}  // Proper heading hierarchy (h1 only once per page)
  </h1>
  <p className={styles.heroSubtitle}>
    {siteConfig.tagline}
  </p>
  <nav 
    className={styles.heroButtons}
    aria-label="Primary navigation"
  >
    <Link 
      to="/"
      aria-label="Explore the book table of contents"
    >
      Explore the Book
    </Link>
    <Link 
      to="/intro"
      aria-label="Start reading the introduction"
    >
      Start Reading
    </Link>
  </nav>
</header>
```

**Accessibility Data Points**:
- `role="banner"` - Identifies hero as page banner landmark
- `aria-label` - Descriptive labels for screen reader navigation
- Semantic HTML (`<header>`, `<h1>`, `<nav>`) - Proper document structure
- Alt text for images - Required by WCAG AA (FR-010)

---

## Performance Metrics

### Asset Loading

**Critical Rendering Path**:
1. HTML (inline critical CSS from Docusaurus)
2. hero-banner.webp (LCP element, must load fast)
3. logo.png (optional, may lazy load if below fold)
4. React hydration (Docusaurus handles)

**Performance Budget**:
- Hero banner image: < 200KB (WebP compressed)
- Logo image: < 50KB (PNG with transparency)
- CSS Modules: < 5KB (scoped styles only)
- Component JS: < 10KB (minimal logic)

**Target Metrics** (from spec SC-004):
- LCP: < 2.5s (hero image is largest contentful paint)
- Total page load: < 2s on 25 Mbps broadband
- Image optimization reduces load time by 60-70% vs unoptimized JPEG

---

## Migration Path (Future Enhancements)

### Potential Dynamic Features (Not in Current Spec)

If future requirements add dynamic content:

**Personalized Greeting**:
```typescript
interface HeroContent {
  // ... existing fields
  personalizedGreeting?: (user: User | null) => string;
}
```

**A/B Testing Variants**:
```typescript
interface HeroVariant {
  id: string;
  title: string;
  tagline: string;
  backgroundImage: string;
  weight: number; // For weighted random selection
}
```

**Analytics Tracking**:
```typescript
interface CTAButton {
  // ... existing fields
  trackingId: string; // For Google Analytics event tracking
  onClickAnalytics?: () => void;
}
```

**Content Management**:
- Move hero content to JSON/YAML config file
- Support multiple language variants (i18n)
- Admin panel for non-technical content updates

**But for current scope**: Static hardcoded content is sufficient and performant.

---

## Summary

**Data Complexity**: Minimal (static configuration only)  
**Entities**: 0 database entities, 1 TypeScript interface  
**State Management**: None required  
**Storage**: Static files only (`/img/` directory)  
**Validation**: Build-time TypeScript type checking  
**Relationships**: Component dependencies only (no data relationships)

**Alignment with Constitution**:
- ✅ No premature abstractions (static is simplest)
- ✅ Performance-first (static assets, no API calls)
- ✅ Accessibility-compliant (semantic HTML, ARIA labels)
- ✅ Type-safe (TypeScript interfaces)
