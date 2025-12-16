---
description: "Task list for Homepage Content Cards feature implementation"
---

# Tasks: Homepage Content Cards

**Input**: Design documents from `/specs/003-homepage-content-cards/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/component-interfaces.ts

**Tests**: Tests are NOT explicitly requested in the feature specification. This implementation follows a manual testing approach with visual validation.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `- [ ] [ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

This is a web application (Docusaurus static site) with frontend components only:
- Components: `src/components/`
- Pages: `src/pages/`
- Data: `src/data/`
- Types: `src/types/`
- Styles: `src/css/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization, dependencies, and type definitions

- [ ] T001 Install framer-motion and react-intersection-observer dependencies via npm
- [ ] T002 [P] Configure Tailwind dark mode sync in tailwind.config.js (darkMode: ['class', '[data-theme="dark"]'])
- [ ] T003 [P] Create TypeScript type definitions in src/types/cards.ts (ModuleCard, InfoCard, LearningOutcome, WeeklyBreakdown, AnimationConfig)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core static data and reusable animation infrastructure that ALL user stories depend on

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T004 Create static data file src/data/modules.ts with 4 module cards (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA)
- [ ] T005 [P] Create static data file src/data/learningOutcomes.ts with 6 learning outcomes
- [ ] T006 [P] Create static data file src/data/weeklyBreakdown.ts with 13-week course structure (6 week ranges)
- [ ] T007 [P] Create static data file src/data/whyPhysicalAI.ts with "Why Physical AI Matters" content
- [ ] T008 Create barrel export src/data/index.ts to export all data files
- [ ] T009 Create AnimatedCard wrapper component in src/components/AnimatedCard/index.tsx with scroll-triggered animations (useInView, Framer Motion variants)
- [ ] T010 [P] Create SectionHeader component in src/components/SectionHeader/index.tsx for animated section titles

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - First-Time Visitor Views Homepage (Priority: P1) 🎯 MVP

**Goal**: Visitor lands on homepage and immediately understands the book's focus through a clean, modern hero banner with enhanced CSS styling

**Independent Test**: Load the homepage and verify that the hero banner displays modern typography with gradient effects, concise messaging, and no large text blocks

### Implementation for User Story 1

- [ ] T011 [US1] Enhance HomepageHero component in src/components/HomepageHero/index.tsx with Framer Motion staggered text animations
- [ ] T012 [US1] Add gradient text effect to hero title in src/components/HomepageHero/index.tsx using Tailwind (bg-gradient-to-r from-blue-600 via-purple-600 to-pink-600 bg-clip-text text-transparent)
- [ ] T013 [US1] Remove or relocate large description text from hero banner in src/components/HomepageHero/index.tsx
- [ ] T014 [US1] Implement reduced motion support in src/components/HomepageHero/index.tsx (disable slide/translate animations, keep fade-in only)
- [ ] T015 [US1] Update hero banner styles in src/components/HomepageHero/styles.module.css with modern typography enhancements (font-weight, letter-spacing, line-height)
- [ ] T016 [US1] Test hero banner responsiveness on mobile (320px), tablet (768px), and desktop (1280px+) screens

**Checkpoint**: At this point, User Story 1 should be fully functional - hero banner displays with modern styling, gradient effects, and no overwhelming text

---

## Phase 4: User Story 2 - Visitor Explores Book Content Overview (Priority: P2)

**Goal**: Visitor scrolls below hero banner to browse book content structure via visual cards showing modules, learning outcomes, and weekly breakdown

**Independent Test**: Scroll below hero banner and verify that modern dashboard-style cards display with visual hierarchy, allowing quick identification of modules within 10 seconds

### Implementation for User Story 2

- [ ] T017 [P] [US2] Create ModuleCard component in src/components/ModuleCard/index.tsx with hover effects (elevation, border glow, icon scale)
- [ ] T018 [P] [US2] Create InfoCard component in src/components/InfoCard/index.tsx for generic info cards (Why Physical AI, Learning Outcomes)
- [ ] T019 [P] [US2] Create WeeklyBreakdownCard component in src/components/WeeklyBreakdownCard/index.tsx with grouped timeline visualization
- [ ] T020 [P] [US2] Create LearningOutcomesCard component in src/components/LearningOutcomesCard/index.tsx with scannable bullet list
- [ ] T021 [US2] Create HomepageContentCards main container in src/components/HomepageContentCards/index.tsx with responsive grid layout
- [ ] T022 [US2] Implement alternating animation directions in src/components/HomepageContentCards/index.tsx (even cards from left, odd cards from right)
- [ ] T023 [US2] Add "Why Physical AI Matters" card to HomepageContentCards using InfoCard component
- [ ] T024 [US2] Add 4 module cards to HomepageContentCards grid with AnimatedCard wrapper
- [ ] T025 [US2] Add learning outcomes card to HomepageContentCards using LearningOutcomesCard component
- [ ] T026 [US2] Add weekly breakdown card to HomepageContentCards using WeeklyBreakdownCard component
- [ ] T027 [US2] Implement card hover effects in ModuleCard using Tailwind (hover:shadow-2xl hover:-translate-y-2 hover:border-primary/50)
- [ ] T028 [US2] Add fallback emoji/icon characters for missing images in all card components (text-based fallback when imagePath fails)
- [ ] T029 [US2] Ensure cards use cohesive color scheme from src/data/modules.ts color definitions
- [ ] T030 [US2] Test content cards section on desktop to verify cards display with proper spacing and shadows

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently - hero banner is enhanced, and content cards display below with full content overview

---

## Phase 5: User Story 3 - Mobile User Accesses Homepage (Priority: P3)

**Goal**: Mobile visitor views homepage with optimal responsive design maintaining readability on smaller screens

**Independent Test**: View homepage on mobile devices (375px, 414px) and verify hero text remains readable and cards stack properly without horizontal scrolling

### Implementation for User Story 3

- [ ] T031 [US3] Implement responsive grid breakpoints in src/components/HomepageContentCards/index.tsx (mobile: 1 col, tablet: 2 col, desktop: 3 col, xl: 4 col)
- [ ] T032 [US3] Add responsive font sizes to hero banner in src/components/HomepageHero/index.tsx (text-5xl md:text-6xl lg:text-7xl)
- [ ] T033 [US3] Ensure card padding and spacing adapts to mobile in src/components/ModuleCard/index.tsx (p-4 sm:p-6)
- [ ] T034 [US3] Test cards stacking vertically on mobile (320px - 640px screens)
- [ ] T035 [US3] Test tablet layout with 2-column grid (640px - 1024px screens)
- [ ] T036 [US3] Test desktop layout with 3-4 column grid (1024px+ screens)
- [ ] T037 [US3] Verify no horizontal scrolling on any screen size from 320px to 4K
- [ ] T038 [US3] Test dark mode toggle across all responsive breakpoints

**Checkpoint**: All user stories should now be independently functional - homepage works perfectly on mobile, tablet, and desktop with full responsive design

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and final integration

- [ ] T039 [P] Update src/pages/index.tsx to integrate HomepageHero and HomepageContentCards components
- [ ] T040 [P] Add accessibility attributes (ARIA labels, semantic HTML) to all card components
- [ ] T041 [P] Implement @media (prefers-reduced-motion: reduce) support across all animated components
- [ ] T042 Optimize Framer Motion bundle size by importing only needed functions (motion, useAnimation, useInView)
- [ ] T043 [P] Test homepage performance with Lighthouse (target: <2s load time, 60fps animations)
- [ ] T044 [P] Test browser compatibility (Chrome, Firefox, Safari, Edge - last 2 versions)
- [ ] T045 Verify WCAG 2.1 Level AA compliance for color contrast ratios in all cards
- [ ] T046 Test keyboard navigation through all cards and hero buttons
- [ ] T047 Run full responsive design validation across 320px, 768px, 1280px, 4K resolutions
- [ ] T048 Verify all animations respect prefers-reduced-motion setting
- [ ] T049 Final visual QA - ensure cohesive design, proper spacing, and visual hierarchy
- [ ] T050 Build production bundle and verify no build errors (npm run build)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User stories can proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Phase 6)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Independently testable (displays content cards below hero)
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Enhances US1 & US2 with responsive design but independently testable

### Within Each User Story

- US1: Hero component enhancement tasks are sequential (T011 → T012 → T013 → T014 → T015)
- US2: Component creation tasks (T017-T020) can run in parallel, then integration tasks (T021-T030)
- US3: Responsive design tasks can run in parallel after US1 & US2 components exist

### Parallel Opportunities

- **Phase 1**: T002, T003 can run in parallel
- **Phase 2**: T005, T006, T007, T010 can run in parallel (different files)
- **User Story 2**: T017, T018, T019, T020 can run in parallel (different card components)
- **Phase 6**: T039, T040, T041, T043, T044 can run in parallel (different concerns)
- **User Stories**: Once Phase 2 completes, all three user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 2 Component Creation

```bash
# Launch all card component creation together (Phase 4, User Story 2):
Task T017: "Create ModuleCard component in src/components/ModuleCard/index.tsx"
Task T018: "Create InfoCard component in src/components/InfoCard/index.tsx"
Task T019: "Create WeeklyBreakdownCard component in src/components/WeeklyBreakdownCard/index.tsx"
Task T020: "Create LearningOutcomesCard component in src/components/LearningOutcomesCard/index.tsx"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (install dependencies, configure Tailwind, create types)
2. Complete Phase 2: Foundational (create all static data, AnimatedCard wrapper) - CRITICAL
3. Complete Phase 3: User Story 1 (enhance hero banner with modern styling)
4. **STOP and VALIDATE**: Test hero banner independently (gradient text, animations, no large text)
5. Deploy/demo if ready (MVP is just enhanced hero banner)

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (Enhanced hero banner - MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo (Content cards below hero)
4. Add User Story 3 → Test independently → Deploy/Demo (Responsive design for mobile)
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together (T001-T010)
2. Once Foundational is done:
   - Developer A: User Story 1 (T011-T016) - Hero banner enhancement
   - Developer B: User Story 2 (T017-T030) - Content cards creation
   - Developer C: User Story 3 (T031-T038) - Responsive design
3. Stories complete and integrate independently via src/pages/index.tsx

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Manual testing approach (no automated tests) with visual validation
- Verify animations work smoothly at 60fps
- Test dark mode thoroughly across all components
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Reduced motion support is critical for accessibility (FR-004)
