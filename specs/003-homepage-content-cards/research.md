# Research: Homepage Content Cards

**Feature**: 003-homepage-content-cards
**Date**: 2025-12-05
**Research Phase**: Phase 0

## Overview

This document captures research findings for implementing modern animated content cards and enhanced hero banner on the Docusaurus homepage. The implementation will use Framer Motion for scroll-triggered animations, custom React components in MDX, and modern CSS hover effects.

## Technology Decisions

### 1. React Components in Docusaurus MDX

**Decision**: Use Docusaurus 3.x MDX v3 with custom React components

**Rationale**:
- Docusaurus 3.x supports MDX v3 natively with built-in React component integration
- Components can be imported directly in `.mdx` files or registered globally via `@theme/MDXComponents`
- MDX allows JSX syntax directly in Markdown files (`.mdx` extension)
- Custom components can be created in `src/components/` and imported with `@site` alias

**Implementation Pattern**:
```tsx
// src/components/HomepageContentCards.tsx
import React from 'react';
import { motion } from 'framer-motion';

export default function HomepageContentCards() {
  return <div>...</div>;
}

// In homepage MDX or custom page component
import HomepageContentCards from '@site/src/components/HomepageContentCards';

<HomepageContentCards />
```

**Alternatives Considered**:
- **Swizzling Docusaurus theme components**: Rejected because it creates maintenance burden when upgrading Docusaurus
- **Pure CSS animations**: Rejected because Framer Motion provides better scroll-based triggers and smoother animations

**References**:
- [Docusaurus MDX and React](https://docusaurus.io/docs/markdown-features/react#mdx-and-react)
- MDX v3 is stricter than CommonMark, requires proper JSX syntax
- Use `@site/` alias for component imports to avoid relative path issues

---

### 2. Framer Motion for Scroll Animations

**Decision**: Use Framer Motion with `useInView` hook for scroll-triggered animations

**Rationale**:
- Industry-standard animation library for React with excellent performance
- `useInView` hook from `react-intersection-observer` provides viewport detection
- Supports scroll-triggered animations that activate when elements enter viewport
- Declarative API with variants for complex animation sequences
- Built-in spring physics for natural motion

**Implementation Pattern**:
```tsx
import { motion, useAnimation } from 'framer-motion';
import { useInView } from 'react-intersection-observer';
import { useEffect } from 'react';

const cardVariants = {
  hidden: { opacity: 0, x: -100 },  // Start off-screen left
  visible: {
    opacity: 1,
    x: 0,
    transition: { duration: 0.6, ease: 'easeOut' }
  }
};

function AnimatedCard({ children, direction = 'left' }) {
  const controls = useAnimation();
  const [ref, inView] = useInView({ triggerOnce: true, threshold: 0.2 });

  useEffect(() => {
    if (inView) {
      controls.start('visible');
    }
  }, [controls, inView]);

  return (
    <motion.div
      ref={ref}
      initial="hidden"
      animate={controls}
      variants={cardVariants}
    >
      {children}
    </motion.div>
  );
}
```

**Key Features Used**:
- **`useInView` hook**: Detects when element enters viewport
  - `triggerOnce: true` - animate only once (not on scroll up)
  - `threshold: 0.2` - trigger when 20% of element visible
  - `rootMargin` - offset trigger point (e.g., `"0px 0px -100px 0px"` triggers 100px before viewport)

- **`useAnimation` + `controls`**: Programmatic animation control triggered by scroll

- **Variants**: Define animation states (hidden/visible) for cleaner code

- **Direction-based animations**:
  - Cards from left: `{ x: -100 }` → `{ x: 0 }`
  - Cards from right: `{ x: 100 }` → `{ x: 0 }`
  - Alternating pattern for visual interest

**Alternatives Considered**:
- **CSS `@keyframes` + Intersection Observer**: Rejected because Framer Motion provides better API and smoother animations
- **GSAP (GreenSock)**: Rejected due to licensing costs for commercial use and larger bundle size
- **React Spring**: Rejected because Framer Motion has better documentation and larger community

**Dependencies**:
```json
{
  "framer-motion": "^11.11.17",
  "react-intersection-observer": "^9.13.1"
}
```

**References**:
- [Framer Motion useInView documentation](https://www.framer.com/motion/use-in-view/)
- [React Intersection Observer](https://github.com/thebuilder/react-intersection-observer)
- Code examples from LogRocket and Stack Overflow demonstrating scroll animations

---

### 3. Shadcn UI Styling Approach

**Decision**: Use Shadcn UI design patterns with Tailwind CSS (NOT using Shadcn components directly)

**Rationale**:
- **Shadcn components are designed for Next.js/Remix**, not Docusaurus
- Shadcn relies on Radix UI primitives which may conflict with Docusaurus theme
- **However**, Shadcn's **design patterns** and **Tailwind class combinations** are excellent references
- We'll extract the visual design principles (shadows, borders, hover effects) and implement with pure Tailwind

**What We'll Adopt from Shadcn**:
1. **Card hover effects**: Elevation changes with shadows
   ```tsx
   className="
     border rounded-lg p-6
     transition-all duration-300
     hover:shadow-xl hover:-translate-y-1
     bg-white dark:bg-gray-900
   "
   ```

2. **Modern glass-morphism effects**:
   ```tsx
   className="
     backdrop-blur-sm bg-white/80 dark:bg-gray-900/80
     border border-gray-200/50 dark:border-gray-700/50
   "
   ```

3. **Smooth transitions**:
   - Use Tailwind `transition-all duration-300`
   - Combine with Framer Motion for scroll entrance
   - Hover effects for interactive feedback

**Alternatives Considered**:
- **Install Shadcn UI components**: Rejected due to Docusaurus incompatibility and unnecessary complexity
- **Material-UI**: Rejected because it adds significant bundle size and doesn't match modern design aesthetic
- **Chakra UI**: Rejected for same reasons as MUI

**Implementation Approach**:
- Pure Tailwind CSS classes following Shadcn design patterns
- Framer Motion for scroll animations
- Custom React components in `src/components/`

**References**:
- [Shadcn UI Hover Card](https://ui.shadcn.com/docs/components/hover-card) - for design inspiration
- Shadcn uses Tailwind + Radix UI + class-variance-authority pattern

---

### 4. Hero Banner Animations

**Decision**: Implement staggered text animations with Framer Motion variants

**Rationale**:
- Hero banner text should animate on page load (not scroll-triggered)
- Staggered animations create professional, engaging experience
- Text elements fade in with subtle slide up motion

**Implementation Pattern**:
```tsx
const heroVariants = {
  hidden: { opacity: 0 },
  visible: {
    opacity: 1,
    transition: {
      staggerChildren: 0.2,  // Delay between each child
      delayChildren: 0.1     // Initial delay before first child
    }
  }
};

const itemVariants = {
  hidden: { opacity: 0, y: 20 },
  visible: {
    opacity: 1,
    y: 0,
    transition: { duration: 0.6, ease: 'easeOut' }
  }
};

function HeroContent() {
  return (
    <motion.div
      initial="hidden"
      animate="visible"
      variants={heroVariants}
    >
      <motion.h1 variants={itemVariants}>Title</motion.h1>
      <motion.p variants={itemVariants}>Tagline</motion.p>
      <motion.div variants={itemVariants}>
        {/* Buttons */}
      </motion.div>
    </motion.div>
  );
}
```

**Key Features**:
- **`staggerChildren`**: Delays each child animation by specified duration
- **`delayChildren`**: Initial delay before animations start
- **Gradient text effect** using Tailwind:
  ```tsx
  className="
    text-5xl font-bold
    bg-gradient-to-r from-blue-600 to-purple-600
    bg-clip-text text-transparent
  "
  ```

---

### 5. Card Hover Effects

**Decision**: Combine CSS transitions with Tailwind utility classes

**Rationale**:
- Pure CSS for performance (no JavaScript overhead)
- Tailwind provides excellent hover variants
- Smooth transitions enhance user experience

**Implementation Pattern**:
```tsx
const cardHoverClasses = `
  group relative
  border border-gray-200 dark:border-gray-800
  rounded-xl overflow-hidden
  bg-white dark:bg-gray-900
  transition-all duration-300 ease-in-out
  hover:shadow-2xl hover:shadow-blue-500/10
  hover:-translate-y-2
  hover:border-blue-500/50
  cursor-pointer
`;

function ContentCard({ title, description, icon }) {
  return (
    <motion.div
      className={cardHoverClasses}
      variants={cardVariants}
      whileHover={{ scale: 1.02 }}  // Framer Motion hover
    >
      {/* Icon with group-hover scale */}
      <div className="text-4xl mb-4 transition-transform duration-300 group-hover:scale-110">
        {icon}
      </div>
      {/* Content */}
      <h3 className="text-xl font-semibold mb-2">{title}</h3>
      <p className="text-gray-600 dark:text-gray-400">{description}</p>
    </motion.div>
  );
}
```

**Hover Effects Breakdown**:
1. **Card elevation**: `hover:shadow-2xl hover:-translate-y-2`
2. **Border glow**: `hover:border-blue-500/50`
3. **Icon scale**: `group-hover:scale-110` (nested element responds to parent hover)
4. **Smooth transitions**: `transition-all duration-300`
5. **Framer Motion scale**: `whileHover={{ scale: 1.02 }}` for subtle zoom

---

### 6. Responsive Grid Layout

**Decision**: CSS Grid with Tailwind responsive utilities

**Rationale**:
- CSS Grid provides best layout control for card grids
- Tailwind breakpoints handle responsive behavior declaratively
- Auto-fit/auto-fill for flexible card count

**Implementation Pattern**:
```tsx
<div className="
  grid grid-cols-1
  sm:grid-cols-2
  lg:grid-cols-3
  xl:grid-cols-4
  gap-6 sm:gap-8
  px-4 sm:px-6 lg:px-8
  max-w-7xl mx-auto
">
  {cards.map((card, index) => (
    <AnimatedCard key={card.id} direction={index % 2 === 0 ? 'left' : 'right'}>
      <ContentCard {...card} />
    </AnimatedCard>
  ))}
</div>
```

**Breakpoints**:
- Mobile (default): 1 column
- Tablet (sm: 640px): 2 columns
- Desktop (lg: 1024px): 3 columns
- Large desktop (xl: 1280px): 4 columns

**Alternating Animation Directions**:
- Even-indexed cards: slide from left (`x: -100`)
- Odd-indexed cards: slide from right (`x: 100`)
- Creates dynamic, engaging appearance

---

### 7. Dark Mode Support

**Decision**: Use Docusaurus built-in dark mode with Tailwind `dark:` variant

**Rationale**:
- Docusaurus provides `data-theme` attribute on `<html>` element
- Tailwind dark mode syncs automatically with Docusaurus color mode
- No additional configuration needed

**Configuration**:
```js
// tailwind.config.js
module.exports = {
  darkMode: ['class', '[data-theme="dark"]'],  // Sync with Docusaurus
  // ...
};
```

**Usage**:
```tsx
className="
  bg-white dark:bg-gray-900
  text-gray-900 dark:text-gray-100
  border-gray-200 dark:border-gray-800
"
```

---

### 8. Performance Optimization

**Decision**: Lazy load Framer Motion and optimize animations

**Rationale**:
- Framer Motion adds ~60KB to bundle
- Not all pages need animations
- Lazy loading improves initial page load

**Implementation**:
```tsx
import dynamic from 'next/dynamic';
import { Suspense, lazy } from 'react';

// Option 1: Next.js dynamic import (if using Next.js pages)
const AnimatedHero = dynamic(() => import('./AnimatedHero'), {
  ssr: false,
  loading: () => <div>Loading...</div>
});

// Option 2: React.lazy for Docusaurus
const AnimatedContentCards = lazy(() => import('./AnimatedContentCards'));

function Homepage() {
  return (
    <Suspense fallback={<div>Loading...</div>}>
      <AnimatedContentCards />
    </Suspense>
  );
}
```

**Optimization Techniques**:
1. **`triggerOnce: true`**: Animate only once, don't re-trigger on scroll up
2. **Reduce motion queries**: Respect user preferences
   ```tsx
   import { useReducedMotion } from 'framer-motion';

   const shouldReduceMotion = useReducedMotion();
   const variants = shouldReduceMotion ? simpleVariants : complexVariants;
   ```
3. **Optimize will-change**: Only apply to animating elements
4. **Use GPU-accelerated properties**: `transform`, `opacity` (NOT `width`, `height`)

---

## Component Architecture

### Component Hierarchy

```
src/pages/index.tsx (or index.mdx)
├── HomepageHero (existing, enhanced)
│   ├── motion.h1 (animated title with gradient)
│   ├── motion.p (animated tagline)
│   └── motion.div (animated buttons)
│
└── HomepageContentCards (NEW)
    ├── SectionHeader (animated)
    ├── CardsGrid
    │   ├── ModuleCard (ROS 2) - animate from left
    │   ├── ModuleCard (Gazebo/Unity) - animate from right
    │   ├── ModuleCard (NVIDIA Isaac) - animate from left
    │   ├── ModuleCard (VLA) - animate from right
    │   ├── WhyPhysicalAICard - animate from bottom
    │   ├── LearningOutcomesCard - animate from bottom
    │   └── WeeklyBreakdownCard - animate from bottom
    └── each card uses: motion.div + cardVariants + useInView
```

### File Structure

```
src/
├── components/
│   ├── HomepageHero/
│   │   ├── index.tsx (enhanced with animations)
│   │   └── styles.module.css (existing)
│   │
│   ├── HomepageContentCards/
│   │   ├── index.tsx (main export)
│   │   ├── ContentCard.tsx (reusable card component)
│   │   ├── AnimatedCard.tsx (wrapper with scroll animations)
│   │   ├── ModuleCard.tsx (specific to modules)
│   │   ├── InfoCard.tsx (for Why Physical AI, Learning Outcomes)
│   │   ├── WeeklyBreakdownCard.tsx (weekly timeline visual)
│   │   └── styles.module.css (custom styles if needed)
│   │
│   └── animations/
│       ├── variants.ts (shared animation variants)
│       └── hooks.ts (custom animation hooks)
│
├── pages/
│   └── index.tsx (homepage using components)
│
└── css/
    └── custom.css (Tailwind imports, global styles)
```

---

## Data Model for Cards

### Module Card Data

```typescript
interface ModuleCardData {
  id: string;
  title: string;
  subtitle: string;
  description: string;
  keyTopics: string[];
  icon: React.ReactNode | string;
  color: {
    primary: string;      // Tailwind color for border/accent
    gradient: string;     // Background gradient
  };
}

const modules: ModuleCardData[] = [
  {
    id: 'ros2',
    title: 'Module 1',
    subtitle: 'The Robotic Nervous System (ROS 2)',
    description: 'Middleware for robot control',
    keyTopics: [
      'ROS 2 Nodes, Topics, and Services',
      'Bridging Python Agents to ROS',
      'URDF for Humanoids'
    ],
    icon: '🤖',
    color: {
      primary: 'blue-500',
      gradient: 'from-blue-500/10 to-purple-500/10'
    }
  },
  // ... other modules
];
```

### Info Card Data

```typescript
interface InfoCardData {
  id: string;
  title: string;
  content: string | React.ReactNode;
  type: 'why-physical-ai' | 'learning-outcomes' | 'weekly-breakdown';
  icon?: React.ReactNode;
}
```

---

## Technical Constraints

### 1. Browser Compatibility
- Target: Modern evergreen browsers (last 2 versions)
- Framer Motion supports: Chrome, Firefox, Safari, Edge
- Intersection Observer API: Widely supported (99%+ browsers)
- CSS Grid: Full support in all modern browsers

### 2. Performance Budgets
- Animation frame rate: Maintain 60fps
- First Contentful Paint: < 1.5s
- Time to Interactive: < 3.5s
- Framer Motion bundle size: ~60KB gzipped (acceptable for enhanced UX)

### 3. Accessibility
- Respect `prefers-reduced-motion` media query
- Ensure animations don't interfere with screen readers
- Maintain keyboard navigation through animated elements
- ARIA labels on interactive cards

---

## Implementation Checklist

Phase 0 (Research) - ✅ COMPLETE:
- [x] Research Docusaurus MDX integration
- [x] Research Framer Motion scroll animations
- [x] Research Shadcn UI design patterns
- [x] Define component architecture
- [x] Document data models

Phase 1 (Design):
- [ ] Create data model for cards content
- [ ] Design animation variants system
- [ ] Define responsive breakpoints
- [ ] Create contracts for component APIs

Phase 2 (Implementation):
- [ ] Install dependencies (framer-motion, react-intersection-observer)
- [ ] Configure Tailwind dark mode sync with Docusaurus
- [ ] Create base AnimatedCard wrapper component
- [ ] Enhance HomepageHero with text animations
- [ ] Build ModuleCard components with hover effects
- [ ] Build InfoCard components
- [ ] Create responsive grid layout
- [ ] Implement alternating animation directions
- [ ] Add reduced motion support
- [ ] Test on mobile/tablet/desktop
- [ ] Performance audit with Lighthouse

---

## Risk Mitigation

### Risk 1: Animation Performance on Low-End Devices
**Mitigation**:
- Use `useReducedMotion` to disable animations on request
- Optimize animations to use only GPU-accelerated properties
- Lazy load Framer Motion library
- Test on mid-range Android devices

### Risk 2: Framer Motion Bundle Size
**Mitigation**:
- Import only needed functions: `import { motion, useAnimation } from 'framer-motion'`
- Consider code-splitting for homepage-specific components
- Current bundle size (~60KB) is acceptable for enhanced UX

### Risk 3: Docusaurus Build Compatibility
**Mitigation**:
- Ensure components are client-side only (no SSR issues)
- Use BrowserOnly wrapper if needed
- Test Docusaurus build process early

### Risk 4: Dark Mode Flickering
**Mitigation**:
- Sync Tailwind dark mode with Docusaurus `data-theme` attribute
- Avoid inline styles that bypass Tailwind dark mode
- Test mode switching thoroughly

---

## References & Resources

### Official Documentation
- [Docusaurus MDX and React](https://docusaurus.io/docs/markdown-features/react)
- [Framer Motion](https://www.framer.com/motion/)
- [React Intersection Observer](https://github.com/thebuilder/react-intersection-observer)
- [Tailwind CSS](https://tailwindcss.com/)

### Code Examples
- [LogRocket: React Scroll Animations](https://blog.logrocket.com/react-scroll-animations-framer-motion/)
- [Framer Motion useInView](https://www.framer.com/motion/use-in-view/)
- [Stack Overflow: Animate when in view](https://stackoverflow.com/questions/58958972)

### Design Inspiration
- [Shadcn UI Components](https://ui.shadcn.com/docs/components)
- Modern card design patterns with shadows and hover effects

---

**Research Completed**: 2025-12-05
**Next Steps**: Proceed to Phase 1 (Data Model & Contracts)
