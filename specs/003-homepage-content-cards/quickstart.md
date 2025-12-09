# Quickstart: Homepage Content Cards

**Feature**: 003-homepage-content-cards
**Date**: 2025-12-05
**Time Estimate**: 3-4 hours for full implementation

## Overview

This quickstart guide provides step-by-step instructions to implement animated content cards and enhanced hero banner on the Docusaurus homepage.

---

## Prerequisites

- Docusaurus 3.9.x site already set up
- Node.js 18+ installed
- Basic knowledge of React and TypeScript
- Tailwind CSS configured in Docusaurus project

---

## Step 1: Install Dependencies (5 minutes)

```bash
npm install framer-motion react-intersection-observer

# Type definitions (if using TypeScript)
npm install --save-dev @types/react
```

**Verify installation**:
```bash
npm list framer-motion react-intersection-observer
```

---

## Step 2: Configure Tailwind Dark Mode (5 minutes)

Update `tailwind.config.js` to sync with Docusaurus dark mode:

```js
// tailwind.config.js
module.exports = {
  content: [
    "./src/**/*.{js,jsx,ts,tsx,md,mdx}",
    "./docs/**/*.{md,mdx}",
  ],
  darkMode: ['class', '[data-theme="dark"]'],  // Sync with Docusaurus
  theme: {
    extend: {
      colors: {
        // Add custom colors if needed
      },
    },
  },
  plugins: [],
  corePlugins: {
    preflight: false,  // Disable Tailwind reset (Docusaurus has its own)
  },
};
```

---

## Step 3: Create Type Definitions (10 minutes)

Create `src/types/cards.ts`:

```typescript
export interface ModuleCard {
  id: string;
  moduleNumber: number;
  title: string;
  subtitle: string;
  focusArea: string;
  description: string;
  keyTopics: string[];
  icon: string;
  color: {
    primary: string;
    gradient: string;
    cardBg: string;
  };
}

export interface LearningOutcome {
  id: string;
  text: string;
  icon?: string;
}

export interface WeekRange {
  startWeek: number;
  endWeek: number;
  title: string;
  topics: string[];
  color: string;
}

export interface WeeklyBreakdown {
  id: string;
  weeks: WeekRange[];
}
```

---

## Step 4: Create Static Data Files (15 minutes)

Create `src/data/modules.ts`:

```typescript
import { ModuleCard } from '../types/cards';

export const modules: ModuleCard[] = [
  {
    id: 'ros2-fundamentals',
    moduleNumber: 1,
    title: 'Module 1',
    subtitle: 'The Robotic Nervous System (ROS 2)',
    focusArea: 'Middleware for robot control',
    description: 'Learn the backbone of modern robotics with ROS 2.',
    keyTopics: [
      'ROS 2 Nodes, Topics, and Services',
      'Bridging Python Agents to ROS',
      'Understanding URDF for humanoids'
    ],
    icon: '🤖',
    color: {
      primary: 'blue-600',
      gradient: 'from-blue-600 to-cyan-500',
      cardBg: 'from-blue-500/5 to-cyan-500/5'
    }
  },
  {
    id: 'gazebo-unity',
    moduleNumber: 2,
    title: 'Module 2',
    subtitle: 'The Digital Twin (Gazebo & Unity)',
    focusArea: 'Physics simulation and environment building',
    description: 'Build photorealistic simulations for robot testing.',
    keyTopics: [
      'Simulating physics, gravity, and collisions',
      'High-fidelity rendering in Unity',
      'Simulating sensors: LiDAR, Depth Cameras, IMUs'
    ],
    icon: '🎮',
    color: {
      primary: 'green-600',
      gradient: 'from-green-600 to-emerald-500',
      cardBg: 'from-green-500/5 to-emerald-500/5'
    }
  },
  {
    id: 'nvidia-isaac',
    moduleNumber: 3,
    title: 'Module 3',
    subtitle: 'The AI-Robot Brain (NVIDIA Isaac™)',
    focusArea: 'Advanced perception and training',
    description: 'Leverage NVIDIA Isaac for AI-powered robotics.',
    keyTopics: [
      'Photorealistic simulation with Isaac Sim',
      'Hardware-accelerated VSLAM and navigation',
      'Path planning for bipedal movement'
    ],
    icon: '🚀',
    color: {
      primary: 'purple-600',
      gradient: 'from-purple-600 to-pink-500',
      cardBg: 'from-purple-500/5 to-pink-500/5'
    }
  },
  {
    id: 'vla',
    moduleNumber: 4,
    title: 'Module 4',
    subtitle: 'Vision-Language-Action (VLA)',
    focusArea: 'The convergence of LLMs and Robotics',
    description: 'Connect language models to robotic actions.',
    keyTopics: [
      'Voice-to-Action with OpenAI Whisper',
      'Cognitive planning with LLMs',
      'Capstone: Autonomous Humanoid project'
    ],
    icon: '💬',
    color: {
      primary: 'orange-600',
      gradient: 'from-orange-600 to-red-500',
      cardBg: 'from-orange-500/5 to-red-500/5'
    }
  }
];
```

Create similar files for other data (learningOutcomes.ts, weeklyBreakdown.ts).

Create barrel export `src/data/index.ts`:

```typescript
export { modules } from './modules';
export { learningOutcomes } from './learningOutcomes';
export { weeklyBreakdown } from './weeklyBreakdown';
```

---

## Step 5: Create Animation Wrapper Component (20 minutes)

Create `src/components/AnimatedCard/index.tsx`:

```typescript
import React, { useEffect } from 'react';
import { motion, useAnimation } from 'framer-motion';
import { useInView } from 'react-intersection-observer';

interface AnimatedCardProps {
  children: React.ReactNode;
  direction?: 'left' | 'right' | 'up' | 'fade';
  delay?: number;
  className?: string;
}

export default function AnimatedCard({
  children,
  direction = 'up',
  delay = 0,
  className = ''
}: AnimatedCardProps) {
  const controls = useAnimation();
  const [ref, inView] = useInView({
    triggerOnce: true,
    threshold: 0.2
  });

  const variants = {
    hidden: {
      opacity: 0,
      x: direction === 'left' ? -100 : direction === 'right' ? 100 : 0,
      y: direction === 'up' ? 50 : 0
    },
    visible: {
      opacity: 1,
      x: 0,
      y: 0,
      transition: {
        duration: 0.6,
        delay: delay / 1000,
        ease: 'easeOut'
      }
    }
  };

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
      variants={variants}
      className={className}
    >
      {children}
    </motion.div>
  );
}
```

---

## Step 6: Create Module Card Component (30 minutes)

Create `src/components/ModuleCard/index.tsx`:

```typescript
import React from 'react';
import { motion } from 'framer-motion';
import { ModuleCard as ModuleCardType } from '@site/src/types/cards';

interface ModuleCardProps {
  module: ModuleCardType;
}

export default function ModuleCard({ module }: ModuleCardProps) {
  return (
    <motion.div
      className={`
        group relative
        border border-gray-200 dark:border-gray-800
        rounded-xl overflow-hidden
        bg-gradient-to-br ${module.color.cardBg}
        dark:bg-gradient-to-br dark:from-gray-900 dark:to-gray-800
        transition-all duration-300 ease-in-out
        hover:shadow-2xl hover:shadow-${module.color.primary}/10
        hover:-translate-y-2
        hover:border-${module.color.primary}/50
        cursor-pointer
        p-6
      `}
      whileHover={{ scale: 1.02 }}
    >
      {/* Icon */}
      <div className="text-5xl mb-4 transition-transform duration-300 group-hover:scale-110">
        {module.icon}
      </div>

      {/* Module Number Badge */}
      <div className={`
        inline-block px-3 py-1 rounded-full text-xs font-semibold mb-3
        bg-gradient-to-r ${module.color.gradient}
        text-white
      `}>
        Module {module.moduleNumber}
      </div>

      {/* Title */}
      <h3 className="text-xl font-bold mb-2 dark:text-white">
        {module.subtitle}
      </h3>

      {/* Focus Area */}
      <p className="text-sm text-gray-600 dark:text-gray-400 mb-3 italic">
        {module.focusArea}
      </p>

      {/* Description */}
      <p className="text-gray-700 dark:text-gray-300 mb-4">
        {module.description}
      </p>

      {/* Key Topics */}
      <div className="space-y-2">
        <p className="text-sm font-semibold text-gray-900 dark:text-gray-100">
          Key Topics:
        </p>
        <ul className="space-y-1">
          {module.keyTopics.map((topic, index) => (
            <li key={index} className="text-sm text-gray-600 dark:text-gray-400 flex items-start">
              <span className={`text-${module.color.primary} mr-2`}>•</span>
              <span>{topic}</span>
            </li>
          ))}
        </ul>
      </div>
    </motion.div>
  );
}
```

---

## Step 7: Create Main Content Cards Component (30 minutes)

Create `src/components/HomepageContentCards/index.tsx`:

```typescript
import React from 'react';
import AnimatedCard from '../AnimatedCard';
import ModuleCard from '../ModuleCard';
import { modules } from '@site/src/data';

export default function HomepageContentCards() {
  return (
    <section className="py-16 px-4 sm:px-6 lg:px-8 bg-white dark:bg-gray-950">
      <div className="max-w-7xl mx-auto">
        {/* Section Header */}
        <div className="text-center mb-12">
          <h2 className="text-4xl font-bold mb-4 dark:text-white">
            Course Modules
          </h2>
          <p className="text-xl text-gray-600 dark:text-gray-400">
            Master Physical AI through four comprehensive modules
          </p>
        </div>

        {/* Modules Grid */}
        <div className="
          grid grid-cols-1
          sm:grid-cols-2
          lg:grid-cols-3
          xl:grid-cols-4
          gap-6 sm:gap-8
        ">
          {modules.map((module, index) => (
            <AnimatedCard
              key={module.id}
              direction={index % 2 === 0 ? 'left' : 'right'}
              delay={index * 100}
            >
              <ModuleCard module={module} />
            </AnimatedCard>
          ))}
        </div>
      </div>
    </section>
  );
}
```

---

## Step 8: Enhance Hero Banner (30 minutes)

Update `src/components/HomepageHero/index.tsx`:

```typescript
import React from 'react';
import { motion } from 'framer-motion';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './styles.module.css';

const heroVariants = {
  hidden: { opacity: 0 },
  visible: {
    opacity: 1,
    transition: {
      staggerChildren: 0.2,
      delayChildren: 0.1
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

export default function HomepageHero() {
  const { siteConfig } = useDocusaurusContext();

  return (
    <header className={styles.heroBanner} role="banner">
      <div className={styles.heroContainer}>
        <motion.div
          className={styles.heroContent}
          initial="hidden"
          animate="visible"
          variants={heroVariants}
        >
          <motion.h1
            className={`
              ${styles.heroTitle}
              text-5xl md:text-6xl lg:text-7xl font-bold
              bg-gradient-to-r from-blue-600 via-purple-600 to-pink-600
              bg-clip-text text-transparent
            `}
            variants={itemVariants}
          >
            {siteConfig.title}
          </motion.h1>

          <motion.p
            className={styles.heroTagline}
            variants={itemVariants}
          >
            {siteConfig.tagline}
          </motion.p>

          <motion.div
            className={styles.heroButtons}
            variants={itemVariants}
          >
            <Link
              className="button button--primary button--lg"
              to="/docs/intro"
            >
              Start Reading
            </Link>
            <Link
              className="button button--secondary button--lg"
              to="/docs/intro"
            >
              Explore the Book
            </Link>
          </motion.div>
        </motion.div>
      </div>
    </header>
  );
}
```

---

## Step 9: Update Homepage (10 minutes)

Update `src/pages/index.tsx`:

```typescript
import React from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageHero from '@site/src/components/HomepageHero';
import HomepageContentCards from '@site/src/components/HomepageContentCards';

export default function Home() {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title={`Home`}
      description={siteConfig.tagline}
    >
      <HomepageHero />
      <main>
        <HomepageContentCards />
      </main>
    </Layout>
  );
}
```

---

## Step 10: Test and Verify (15 minutes)

1. **Start dev server**:
   ```bash
   npm start
   ```

2. **Test animations**:
   - Scroll down to see cards animate into view
   - Hover over cards to see elevation effects
   - Toggle dark mode to verify styling

3. **Test responsive design**:
   - Open DevTools
   - Test on mobile (375px), tablet (768px), desktop (1280px)
   - Verify cards stack properly on small screens

4. **Test accessibility**:
   - Navigate with keyboard (Tab key)
   - Verify focus indicators are visible
   - Check screen reader compatibility

---

## Troubleshooting

### Issue: Animations not working
**Solution**: Verify Framer Motion is installed correctly:
```bash
npm list framer-motion
```

### Issue: Tailwind classes not applying
**Solution**: Ensure `tailwind.config.js` content paths include `src/**/*.{ts,tsx}`:
```js
content: ["./src/**/*.{js,jsx,ts,tsx,md,mdx}"]
```

### Issue: Dark mode not syncing
**Solution**: Update `tailwind.config.js`:
```js
darkMode: ['class', '[data-theme="dark"]']
```

### Issue: Hydration errors in Docusaurus
**Solution**: Wrap animated components in `BrowserOnly` if needed:
```tsx
import BrowserOnly from '@docusaurus/BrowserOnly';

<BrowserOnly fallback={<div>Loading...</div>}>
  {() => <HomepageContentCards />}
</BrowserOnly>
```

---

## Next Steps

1. Add learning outcomes card component
2. Add weekly breakdown timeline component
3. Implement "Why Physical AI Matters" card
4. Add more sophisticated hover effects
5. Performance optimization (lazy loading, code splitting)

---

## Time Breakdown

- Step 1-2: 10 min (Dependencies & config)
- Step 3-4: 25 min (Types & data)
- Step 5-6: 50 min (Animation & card components)
- Step 7-8: 60 min (Main components & hero)
- Step 9-10: 25 min (Integration & testing)

**Total**: ~3 hours

---

## Resources

- [Framer Motion Docs](https://www.framer.com/motion/)
- [Docusaurus React Integration](https://docusaurus.io/docs/markdown-features/react)
- [Tailwind CSS Docs](https://tailwindcss.com/)
- [React Intersection Observer](https://github.com/thebuilder/react-intersection-observer)

**Quickstart Completed**: 2025-12-05
