# Data Model: Homepage Content Cards

**Feature**: 003-homepage-content-cards
**Date**: 2025-12-05

## Overview

This document defines the data structures for homepage content cards, including modules, learning outcomes, weekly breakdown, and card metadata. All data is **statically defined** (no database or API required) as this is a static Docusaurus site.

---

## Core Entities

### 1. ModuleCard

Represents one of the four main course modules displayed as cards.

```typescript
interface ModuleCard {
  id: string;                    // Unique identifier (kebab-case)
  moduleNumber: number;          // 1-4
  title: string;                 // Module title
  subtitle: string;              // Brief descriptor (e.g., "The Robotic Nervous System")
  focusArea: string;             // What this module teaches
  description: string;           // 1-2 sentence overview
  keyTopics: string[];           // 3-4 bullet points
  icon: string;                  // Emoji or icon component name
  color: ModuleColor;            // Color theme for card
  imagePath?: string;            // Optional: path to module illustration
}

interface ModuleColor {
  primary: string;               // Tailwind color class (e.g., "blue-500")
  secondary: string;             // Complementary color
  gradient: string;              // Tailwind gradient (e.g., "from-blue-500 to-purple-500")
  cardBg: string;                // Card background gradient (subtle)
}

// Example instance
const module1: ModuleCard = {
  id: 'ros2-fundamentals',
  moduleNumber: 1,
  title: 'Module 1: ROS 2 Fundamentals',
  subtitle: 'The Robotic Nervous System',
  focusArea: 'Middleware for robot control',
  description: 'Learn the backbone of modern robotics with ROS 2, the industry-standard middleware for building intelligent robot systems.',
  keyTopics: [
    'ROS 2 Nodes, Topics, and Services',
    'Bridging Python Agents to ROS controllers using rclpy',
    'Understanding URDF (Unified Robot Description Format) for humanoids'
  ],
  icon: '🤖',
  color: {
    primary: 'blue-600',
    secondary: 'blue-400',
    gradient: 'from-blue-600 to-cyan-500',
    cardBg: 'from-blue-500/5 to-cyan-500/5'
  },
  imagePath: '/img/modules/ros2-icon.svg'  // Optional
};
```

**Validation Rules**:
- `id`: Must be unique, lowercase, kebab-case
- `moduleNumber`: Integer 1-4
- `keyTopics`: 3-4 items (enforce in component)
- `description`: Max 200 characters for consistent card heights

---

### 2. InfoCard

Generic card for "Why Physical AI Matters" and "Learning Outcomes" sections.

```typescript
type InfoCardType = 'why-physical-ai' | 'learning-outcomes' | 'weekly-breakdown';

interface InfoCard {
  id: string;
  type: InfoCardType;
  title: string;
  content: string | InfoCardContent;  // String or structured content
  icon?: string;                       // Emoji or icon
  color: CardColor;
}

interface InfoCardContent {
  introduction?: string;
  items: InfoCardItem[];
  conclusion?: string;
}

interface InfoCardItem {
  title?: string;           // Optional heading for item
  description: string;      // Main text
  icon?: string;            // Optional icon per item
}

interface CardColor {
  primary: string;
  gradient: string;
  cardBg: string;
}

// Example: Why Physical AI Matters
const whyPhysicalAICard: InfoCard = {
  id: 'why-physical-ai',
  type: 'why-physical-ai',
  title: 'Why Physical AI Matters',
  icon: '🌟',
  content: {
    introduction: 'Humanoid robots are poised to excel in our human-centered world because they share our physical form and can be trained with abundant data from interacting in human environments.',
    items: [
      {
        icon: '🏗️',
        description: 'This represents a significant transition from AI models confined to digital environments to embodied intelligence that operates in physical space.'
      }
    ]
  },
  color: {
    primary: 'purple-600',
    gradient: 'from-purple-600 to-pink-500',
    cardBg: 'from-purple-500/5 to-pink-500/5'
  }
};
```

---

### 3. LearningOutcome

Individual learning outcome displayed in the Learning Outcomes card.

```typescript
interface LearningOutcome {
  id: string;
  text: string;
  icon?: string;
  category?: 'technical' | 'conceptual' | 'practical';
}

// Learning Outcomes data
const learningOutcomes: LearningOutcome[] = [
  {
    id: 'physical-ai-principles',
    text: 'Understand Physical AI principles and embodied intelligence',
    icon: '🧠',
    category: 'conceptual'
  },
  {
    id: 'ros2-mastery',
    text: 'Master ROS 2 (Robot Operating System) for robotic control',
    icon: '⚙️',
    category: 'technical'
  },
  {
    id: 'simulation',
    text: 'Simulate robots with Gazebo and Unity',
    icon: '🎮',
    category: 'practical'
  },
  {
    id: 'nvidia-isaac',
    text: 'Develop with NVIDIA Isaac AI robot platform',
    icon: '🚀',
    category: 'technical'
  },
  {
    id: 'humanoid-design',
    text: 'Design humanoid robots for natural interactions',
    icon: '🤝',
    category: 'practical'
  },
  {
    id: 'gpt-integration',
    text: 'Integrate GPT models for conversational robotics',
    icon: '💬',
    category: 'technical'
  }
];
```

---

### 4. WeeklyBreakdown

Represents the 13-week course structure with topic groupings.

```typescript
interface WeeklyBreakdown {
  id: string;
  weeks: WeekRange[];
}

interface WeekRange {
  startWeek: number;
  endWeek: number;
  title: string;
  topics: string[];
  color: string;        // Tailwind color for visual grouping
}

const weeklyBreakdown: WeeklyBreakdown = {
  id: 'weekly-breakdown',
  weeks: [
    {
      startWeek: 1,
      endWeek: 2,
      title: 'Introduction to Physical AI',
      topics: [
        'Foundations of Physical AI and embodied intelligence',
        'From digital AI to robots that understand physical laws',
        'Overview of humanoid robotics landscape',
        'Sensor systems: LIDAR, cameras, IMUs, force/torque sensors'
      ],
      color: 'blue-500'
    },
    {
      startWeek: 3,
      endWeek: 5,
      title: 'ROS 2 Fundamentals',
      topics: [
        'ROS 2 architecture and core concepts',
        'Nodes, topics, services, and actions',
        'Building ROS 2 packages with Python',
        'Launch files and parameter management'
      ],
      color: 'green-500'
    },
    {
      startWeek: 6,
      endWeek: 7,
      title: 'Robot Simulation with Gazebo',
      topics: [
        'Gazebo simulation environment setup',
        'URDF and SDF robot description formats',
        'Physics simulation and sensor simulation',
        'Introduction to Unity for robot visualization'
      ],
      color: 'yellow-500'
    },
    {
      startWeek: 8,
      endWeek: 10,
      title: 'NVIDIA Isaac Platform',
      topics: [
        'NVIDIA Isaac SDK and Isaac Sim',
        'AI-powered perception and manipulation',
        'Reinforcement learning for robot control',
        'Sim-to-real transfer techniques'
      ],
      color: 'purple-500'
    },
    {
      startWeek: 11,
      endWeek: 12,
      title: 'Humanoid Robot Development',
      topics: [
        'Humanoid robot kinematics and dynamics',
        'Bipedal locomotion and balance control',
        'Manipulation and grasping with humanoid hands',
        'Natural human-robot interaction design'
      ],
      color: 'red-500'
    },
    {
      startWeek: 13,
      endWeek: 13,
      title: 'Conversational Robotics',
      topics: [
        'Integrating GPT models for conversational AI in robots',
        'Speech recognition and natural language understanding',
        'Multi-modal interaction: speech, gesture, vision'
      ],
      color: 'pink-500'
    }
  ]
};
```

---

### 5. AnimationConfig

Configuration for scroll animations per card.

```typescript
type AnimationDirection = 'left' | 'right' | 'up' | 'down' | 'fade';

interface AnimationConfig {
  direction: AnimationDirection;
  delay?: number;              // Stagger delay (ms)
  duration?: number;           // Animation duration (seconds)
  threshold?: number;          // Intersection observer threshold (0-1)
  triggerOnce?: boolean;       // Animate only once
}

// Default animation config
const defaultAnimationConfig: AnimationConfig = {
  direction: 'up',
  delay: 0,
  duration: 0.6,
  threshold: 0.2,
  triggerOnce: true
};

// Module cards alternate left/right
const moduleAnimations: Record<number, AnimationConfig> = {
  1: { direction: 'left', delay: 0, duration: 0.6, threshold: 0.2, triggerOnce: true },
  2: { direction: 'right', delay: 100, duration: 0.6, threshold: 0.2, triggerOnce: true },
  3: { direction: 'left', delay: 200, duration: 0.6, threshold: 0.2, triggerOnce: true },
  4: { direction: 'right', delay: 300, duration: 0.6, threshold: 0.2, triggerOnce: true }
};
```

---

## Data Storage & Access

### Static Data Files

All card data is stored in TypeScript/JavaScript files as constants (no external API or database).

**File Structure**:
```
src/data/
├── modules.ts          # Module card data (ModuleCard[])
├── learningOutcomes.ts # Learning outcomes (LearningOutcome[])
├── weeklyBreakdown.ts  # Weekly breakdown (WeeklyBreakdown)
├── whyPhysicalAI.ts    # Why Physical AI card data
└── index.ts            # Barrel export
```

**Example Export**:
```typescript
// src/data/modules.ts
import { ModuleCard } from '../types/cards';

export const modules: ModuleCard[] = [
  {
    id: 'ros2-fundamentals',
    moduleNumber: 1,
    title: 'Module 1: ROS 2 Fundamentals',
    // ... rest of data
  },
  // ... other modules
];

// src/data/index.ts
export { modules } from './modules';
export { learningOutcomes } from './learningOutcomes';
export { weeklyBreakdown } from './weeklyBreakdown';
export { whyPhysicalAICard } from './whyPhysicalAI';
```

### Component Usage

Components import data directly from data files:

```typescript
// src/components/HomepageContentCards/index.tsx
import { modules, learningOutcomes, weeklyBreakdown } from '@site/src/data';

export default function HomepageContentCards() {
  return (
    <section>
      <ModulesGrid modules={modules} />
      <InfoCardsSection
        outcomes={learningOutcomes}
        breakdown={weeklyBreakdown}
      />
    </section>
  );
}
```

---

## TypeScript Type Definitions

All types are defined in a shared types file:

**src/types/cards.ts**:
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
  color: ModuleColor;
  imagePath?: string;
}

export interface ModuleColor {
  primary: string;
  secondary: string;
  gradient: string;
  cardBg: string;
}

export type InfoCardType = 'why-physical-ai' | 'learning-outcomes' | 'weekly-breakdown';

export interface InfoCard {
  id: string;
  type: InfoCardType;
  title: string;
  content: string | InfoCardContent;
  icon?: string;
  color: CardColor;
}

export interface InfoCardContent {
  introduction?: string;
  items: InfoCardItem[];
  conclusion?: string;
}

export interface InfoCardItem {
  title?: string;
  description: string;
  icon?: string;
}

export interface CardColor {
  primary: string;
  gradient: string;
  cardBg: string;
}

export interface LearningOutcome {
  id: string;
  text: string;
  icon?: string;
  category?: 'technical' | 'conceptual' | 'practical';
}

export interface WeeklyBreakdown {
  id: string;
  weeks: WeekRange[];
}

export interface WeekRange {
  startWeek: number;
  endWeek: number;
  title: string;
  topics: string[];
  color: string;
}

export type AnimationDirection = 'left' | 'right' | 'up' | 'down' | 'fade';

export interface AnimationConfig {
  direction: AnimationDirection;
  delay?: number;
  duration?: number;
  threshold?: number;
  triggerOnce?: boolean;
}
```

---

## Validation & Constraints

### Data Validation Rules

1. **Module Cards**:
   - Must have exactly 4 module cards (moduleNumber 1-4)
   - Each `id` must be unique
   - `keyTopics` must have 3-4 items
   - `description` max 200 characters

2. **Learning Outcomes**:
   - Minimum 3, maximum 10 outcomes
   - Each outcome text max 100 characters

3. **Weekly Breakdown**:
   - Must cover weeks 1-13 (no gaps)
   - Each week range must have `endWeek >= startWeek`
   - Total weeks must equal 13

### Runtime Validation (Optional)

```typescript
// src/utils/validateCardData.ts
import { z } from 'zod';

const ModuleCardSchema = z.object({
  id: z.string().min(1),
  moduleNumber: z.number().int().min(1).max(4),
  title: z.string().min(1),
  subtitle: z.string().min(1),
  focusArea: z.string().min(1),
  description: z.string().max(200),
  keyTopics: z.array(z.string()).min(3).max(4),
  icon: z.string().min(1),
  color: z.object({
    primary: z.string(),
    secondary: z.string(),
    gradient: z.string(),
    cardBg: z.string()
  }),
  imagePath: z.string().optional()
});

export function validateModules(modules: unknown[]) {
  const result = z.array(ModuleCardSchema).length(4).safeParse(modules);
  if (!result.success) {
    console.error('Module validation errors:', result.error);
    throw new Error('Invalid module card data');
  }
  return result.data;
}
```

---

## Summary

All data for homepage content cards is:
- **Statically defined** in TypeScript files (`src/data/`)
- **Strongly typed** using TypeScript interfaces
- **Validated** at build time (TypeScript) and optionally at runtime (Zod)
- **Imported directly** by components (no API calls required)
- **Easy to update** by editing data files

No database, no API, no CMS—pure static data perfect for Docusaurus static site generation.

---

**Data Model Completed**: 2025-12-05
**Next Steps**: Generate API contracts (component interfaces) and quickstart guide
