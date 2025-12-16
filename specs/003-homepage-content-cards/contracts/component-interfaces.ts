/**
 * Component Contracts: Homepage Content Cards
 * Feature: 003-homepage-content-cards
 * Date: 2025-12-05
 *
 * This file defines the TypeScript interfaces for all React components
 * used in the Homepage Content Cards feature.
 */

import { ReactNode } from 'react';
import { ModuleCard, InfoCard, LearningOutcome, WeeklyBreakdown, AnimationConfig } from '../../../src/types/cards';

// ============================================================================
// ANIMATION COMPONENTS
// ============================================================================

/**
 * AnimatedCard - Wrapper component that adds scroll-triggered animations
 *
 * Usage:
 * ```tsx
 * <AnimatedCard direction="left" delay={100}>
 *   <ContentCard {...data} />
 * </AnimatedCard>
 * ```
 */
export interface AnimatedCardProps {
  children: ReactNode;
  direction?: 'left' | 'right' | 'up' | 'down' | 'fade';
  delay?: number;           // Delay before animation starts (ms)
  duration?: number;        // Animation duration (seconds)
  threshold?: number;       // Intersection observer threshold (0-1)
  triggerOnce?: boolean;    // Animate only once (default: true)
  className?: string;
}

/**
 * FadeInWhenVisible - Simple fade-in animation when element enters viewport
 *
 * Usage:
 * ```tsx
 * <FadeInWhenVisible delay={200}>
 *   <h2>Section Title</h2>
 * </FadeInWhenVisible>
 * ```
 */
export interface FadeInWhenVisibleProps {
  children: ReactNode;
  delay?: number;
  className?: string;
}

// ============================================================================
// HERO BANNER COMPONENTS
// ============================================================================

/**
 * HomepageHero - Enhanced hero banner with animated text
 *
 * Features:
 * - Staggered text animations (title, tagline, description)
 * - Gradient text effect on title
 * - Animated action buttons
 *
 * Usage:
 * ```tsx
 * <HomepageHero />
 * ```
 */
export interface HomepageHeroProps {
  // No props needed - uses site config for title/tagline
}

/**
 * AnimatedHeroText - Individual animated text element in hero
 */
export interface AnimatedHeroTextProps {
  children: ReactNode;
  delay?: number;
  variant?: 'title' | 'subtitle' | 'description';
  className?: string;
}

// ============================================================================
// CONTENT CARD COMPONENTS
// ============================================================================

/**
 * ContentCard - Base card component with hover effects
 *
 * Features:
 * - Elevation shadow on hover
 * - Border glow effect
 * - Icon scale animation
 * - Responsive layout
 *
 * Usage:
 * ```tsx
 * <ContentCard
 *   title="Module Title"
 *   description="Description"
 *   icon="🤖"
 *   color={{ primary: 'blue-500', ... }}
 * />
 * ```
 */
export interface ContentCardProps {
  title: string;
  description: string;
  icon?: string | ReactNode;
  color?: {
    primary: string;
    gradient?: string;
    cardBg?: string;
  };
  children?: ReactNode;
  className?: string;
  onClick?: () => void;
}

/**
 * ModuleCard - Card specifically for displaying course modules
 *
 * Usage:
 * ```tsx
 * <ModuleCard module={module1} />
 * ```
 */
export interface ModuleCardProps {
  module: ModuleCard;
  animationDelay?: number;
  animationDirection?: 'left' | 'right';
}

/**
 * InfoCard - Card for "Why Physical AI" and "Learning Outcomes"
 *
 * Usage:
 * ```tsx
 * <InfoCard data={whyPhysicalAICard} />
 * ```
 */
export interface InfoCardComponentProps {
  data: InfoCard;
  animationDelay?: number;
}

/**
 * LearningOutcomesCard - Displays all learning outcomes
 *
 * Usage:
 * ```tsx
 * <LearningOutcomesCard outcomes={learningOutcomes} />
 * ```
 */
export interface LearningOutcomesCardProps {
  outcomes: LearningOutcome[];
  animationDelay?: number;
}

/**
 * WeeklyBreakdownCard - Visual timeline of 13-week course
 *
 * Usage:
 * ```tsx
 * <WeeklyBreakdownCard breakdown={weeklyBreakdown} />
 * ```
 */
export interface WeeklyBreakdownCardProps {
  breakdown: WeeklyBreakdown;
  animationDelay?: number;
}

// ============================================================================
// LAYOUT COMPONENTS
// ============================================================================

/**
 * HomepageContentCards - Main container for all content cards
 *
 * Features:
 * - Section header with animation
 * - Responsive grid layout
 * - Alternating animation directions
 * - Dark mode support
 *
 * Usage:
 * ```tsx
 * <HomepageContentCards />
 * ```
 */
export interface HomepageContentCardsProps {
  className?: string;
}

/**
 * CardsGrid - Responsive grid container for cards
 *
 * Usage:
 * ```tsx
 * <CardsGrid columns={{ mobile: 1, tablet: 2, desktop: 3 }}>
 *   {cards.map((card) => <Card key={card.id} {...card} />)}
 * </CardsGrid>
 * ```
 */
export interface CardsGridProps {
  children: ReactNode;
  columns?: {
    mobile?: number;    // cols on mobile (default: 1)
    tablet?: number;    // cols on tablet (default: 2)
    desktop?: number;   // cols on desktop (default: 3)
    wide?: number;      // cols on xl screens (default: 4)
  };
  gap?: 'sm' | 'md' | 'lg';
  className?: string;
}

/**
 * SectionHeader - Animated section header
 *
 * Usage:
 * ```tsx
 * <SectionHeader
 *   title="Course Modules"
 *   subtitle="Explore the four core modules"
 * />
 * ```
 */
export interface SectionHeaderProps {
  title: string;
  subtitle?: string;
  className?: string;
}

// ============================================================================
// UTILITY TYPES
// ============================================================================

/**
 * Card hover state for interactive feedback
 */
export type CardHoverState = 'idle' | 'hovering';

/**
 * Animation variant definitions
 */
export interface AnimationVariants {
  hidden: {
    opacity: number;
    x?: number;
    y?: number;
    scale?: number;
  };
  visible: {
    opacity: number;
    x?: number;
    y?: number;
    scale?: number;
    transition?: {
      duration?: number;
      delay?: number;
      ease?: string | number[];
      staggerChildren?: number;
    };
  };
}

/**
 * Responsive breakpoint configuration
 */
export interface ResponsiveConfig {
  mobile: number;
  tablet: number;
  desktop: number;
  wide: number;
}

// ============================================================================
// COMPONENT COMPOSITION PATTERNS
// ============================================================================

/**
 * Example: Full homepage composition
 *
 * ```tsx
 * import HomepageHero from '@site/src/components/HomepageHero';
 * import HomepageContentCards from '@site/src/components/HomepageContentCards';
 *
 * export default function Home() {
 *   return (
 *     <Layout>
 *       <HomepageHero />
 *       <main>
 *         <HomepageContentCards />
 *       </main>
 *     </Layout>
 *   );
 * }
 * ```
 */

/**
 * Example: Custom card grid
 *
 * ```tsx
 * import { CardsGrid, AnimatedCard, ContentCard } from '@site/src/components';
 * import { modules } from '@site/src/data';
 *
 * function ModulesSection() {
 *   return (
 *     <CardsGrid columns={{ mobile: 1, tablet: 2, desktop: 3 }}>
 *       {modules.map((module, index) => (
 *         <AnimatedCard
 *           key={module.id}
 *           direction={index % 2 === 0 ? 'left' : 'right'}
 *           delay={index * 100}
 *         >
 *           <ModuleCard module={module} />
 *         </AnimatedCard>
 *       ))}
 *     </CardsGrid>
 *   );
 * }
 * ```
 */

// ============================================================================
// PROP TYPE GUARDS
// ============================================================================

/**
 * Type guard to check if content is structured InfoCardContent
 */
export function isInfoCardContent(content: unknown): content is import('../../../src/types/cards').InfoCardContent {
  return (
    typeof content === 'object' &&
    content !== null &&
    'items' in content &&
    Array.isArray((content as any).items)
  );
}

/**
 * Type guard for ReactNode vs string icon
 */
export function isReactNodeIcon(icon: unknown): icon is ReactNode {
  return typeof icon !== 'string';
}

// ============================================================================
// EXPORTS
// ============================================================================

export type {
  // Animation components
  AnimatedCardProps,
  FadeInWhenVisibleProps,

  // Hero components
  HomepageHeroProps,
  AnimatedHeroTextProps,

  // Card components
  ContentCardProps,
  ModuleCardProps,
  InfoCardComponentProps,
  LearningOutcomesCardProps,
  WeeklyBreakdownCardProps,

  // Layout components
  HomepageContentCardsProps,
  CardsGridProps,
  SectionHeaderProps,

  // Utility types
  CardHoverState,
  AnimationVariants,
  ResponsiveConfig,
};

export {
  isInfoCardContent,
  isReactNodeIcon,
};
