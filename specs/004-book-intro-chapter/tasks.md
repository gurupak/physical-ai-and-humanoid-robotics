# Tasks: Book Introduction Chapter

**Input**: Design documents from `/specs/004-book-intro-chapter/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, quickstart.md

**Tests**: No test tasks included (not requested in specification - manual user testing will be performed post-implementation)

**Organization**: Tasks organized by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- Docusaurus project: `docs/` for content, `static/img/` for images, `docusaurus.config.ts` for configuration
- Introduction chapter: `docs/intro.md`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure verification

- [X] T001 Verify Docusaurus 3.9.x installation and verify @docusaurus/theme-mermaid is configured in docusaurus.config.ts
- [X] T002 [P] Verify docs/ directory structure exists and is properly configured in docusaurus.config.ts
- [X] T003 [P] Test Mermaid diagram rendering in local development server (npm start)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before content writing can begin

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T004 Read existing docs/intro.md to understand current content structure
- [X] T005 [P] Create backup of existing docs/intro.md if substantive content exists
- [X] T006 [P] Review quickstart.md templates and tone guidelines in specs/004-book-intro-chapter/quickstart.md

**Checkpoint**: Foundation ready - content writing can now begin

---

## Phase 3: User Story 1 - First-Time Reader Understands Book Value (Priority: P1) 🎯 MVP

**Goal**: Create introduction content that allows first-time readers to quickly understand what the book covers, who it's for, and what they'll achieve within 5 minutes of reading

**Independent Test**: Have 5-10 target readers (beginner to intermediate in robotics) read the introduction and answer: "Do you understand what this book is about?" and "Would you continue reading?" Success = 90% yes on both questions

### Implementation for User Story 1

- [X] T007 [US1] Add frontmatter metadata to docs/intro.md with id, title, sidebar_label, sidebar_position: 1, and custom props for difficulty (Beginner) and readingTime (18 minutes)
- [X] T008 [P] [US1] Write "Why Physical AI Matters" section (2-3 paragraphs, ~300 words) in docs/intro.md using encouraging conversational tone with detailed technical definitions
- [X] T009 [P] [US1] Write "What This Book Covers" section with 4 module descriptions in docs/intro.md
- [X] T010 [US1] Create Mermaid flowchart showing module relationships (ROS 2 → Simulation → NVIDIA Isaac → VLA → Deployment) in docs/intro.md
- [X] T011 [P] [US1] Write "Who This Book Is For" section with 3 target audience profiles in docs/intro.md
- [X] T012 [P] [US1] Write "Learning Outcomes" section with 6-8 measurable skill statements starting with action verbs in docs/intro.md
- [X] T013 [US1] Add clear navigation link to Chapter 1 at the end of docs/intro.md
- [X] T014 [US1] Review and verify all technical terms have 3-4 sentence definitions with examples and analogies in docs/intro.md
- [X] T015 [US1] Verify word count is 2000-3000 words and reading time is 15-20 minutes

**Checkpoint**: At this point, User Story 1 should be complete - readers can understand book value and scope

---

## Phase 4: User Story 2 - Reader Understands Course Structure (Priority: P2)

**Goal**: Provide readers with complete understanding of how the book is organized, what topics each chapter covers, and the recommended 13-week learning path

**Independent Test**: Ask readers to create a 13-week study plan based on the introduction. Success = readers can allocate chapters to weeks and identify prerequisite chains

### Implementation for User Story 2

- [X] T016 [P] [US2] Write "Course Structure" section with 13-week timeline breakdown in docs/intro.md
- [X] T017 [US2] Create Mermaid Gantt chart showing 13-week learning progression with week ranges for each phase in docs/intro.md
- [X] T018 [P] [US2] Write "Prerequisites" section with required skills (Python basics, Linux CLI) and optional skills in docs/intro.md
- [X] T019 [US2] Add time commitment estimate (5-10 hours per week) and clarify timeline flexibility in docs/intro.md
- [X] T020 [US2] Verify week-by-week descriptions show clear progression from ROS 2 basics through simulation, GPU acceleration, and VLA models

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently - readers understand value AND can plan learning journey

---

## Phase 5: User Story 3 - Reader Connects with Real-World Applications (Priority: P3)

**Goal**: Help readers understand why Physical AI matters and how skills learned apply to real-world problems, industry trends, and career opportunities to build motivation

**Independent Test**: Ask readers to name 3 real-world applications of Physical AI after reading the introduction. Success = 80% can name relevant applications (autonomous vehicles, humanoid assistants, warehouse automation, etc.)

### Implementation for User Story 3

- [X] T021 [P] [US3] Write "How to Use This Book" section with skip-ahead guidance for different experience levels in docs/intro.md
- [X] T022 [P] [US3] Add specific chapter recommendations for readers with ROS 2 experience, simulation experts, and AI/ML background in docs/intro.md
- [X] T023 [US3] Expand "Why Physical AI Matters" section to include specific industry applications (automotive, logistics, healthcare, manufacturing) in docs/intro.md
- [X] T024 [US3] Add career opportunities and industry momentum context to "Why Physical AI Matters" section in docs/intro.md
- [X] T025 [US3] Add concrete project examples showing what readers will build throughout the book in docs/intro.md
- [X] T026 [US3] Verify all real-world applications mentioned are current and not hyper-specific to avoid rapid obsolescence

**Checkpoint**: All user stories should now be independently functional - introduction is complete

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Final improvements that affect the entire introduction chapter

- [X] T027 [P] Test rendering in local Docusaurus build (npm start) and verify no Mermaid diagram errors
- [X] T028 [P] Verify all 7 required sections present in order (Why, What, Who, Learning Outcomes, Structure, Prerequisites, How to Use)
- [X] T029 Proofread entire introduction for tone consistency (second-person "you", active voice, encouraging language)
- [X] T030 [P] Verify all technical terms are defined inline on first use with 3-4 sentence format
- [X] T031 [P] Check accessibility: headings follow hierarchy (H1 → H2 → H3), diagrams have descriptive text
- [X] T032 Validate against success criteria SC-001 through SC-008 from spec.md
- [X] T033 [P] Run Docusaurus production build (npm run build) to verify no errors
- [ ] T034 Manual user testing with 5-10 target readers to verify 90% comprehension threshold (SC-005)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User stories can proceed sequentially in priority order (P1 → P2 → P3)
  - Each story adds content incrementally to docs/intro.md
- **Polish (Phase 6)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - Writes core sections (Why, What, Who, Learning Outcomes)
- **User Story 2 (P2)**: Can start after US1 - Adds structure sections (Course Structure, Prerequisites)
- **User Story 3 (P3)**: Can start after US1 and US2 - Enhances existing sections with real-world context and navigation guidance

### Within Each User Story

- Frontmatter first (T007) before any content writing
- Sections can be written in parallel if marked [P]
- Diagrams should be added after related section text
- Validation tasks after content completion

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel (T002, T003)
- All Foundational tasks marked [P] can run in parallel (T005, T006)
- Within US1: T008, T009, T011, T012 can run in parallel (different sections)
- Within US2: T016, T018 can run in parallel (different sections)
- Within US3: T021, T022, T023, T024, T025 can run in parallel (enhancing different sections)
- Within Polish: T027, T028, T030, T031, T033 can run in parallel (different validation checks)

---

## Parallel Example: User Story 1

```bash
# After frontmatter (T007), launch section writing in parallel:
Task T008: Write "Why Physical AI Matters" section
Task T009: Write "What This Book Covers" section
Task T011: Write "Who This Book Is For" section
Task T012: Write "Learning Outcomes" section

# These can all be written simultaneously as they don't depend on each other
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (verify Docusaurus and Mermaid)
2. Complete Phase 2: Foundational (read existing content, review templates)
3. Complete Phase 3: User Story 1 (core introduction content)
4. **STOP and VALIDATE**: Test that first-time readers understand book value
5. Commit and optionally deploy for early feedback

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Commit (MVP - readers understand what the book is!)
3. Add User Story 2 → Test independently → Commit (readers can now plan their learning journey)
4. Add User Story 3 → Test independently → Commit (readers connected with real-world applications)
5. Polish Phase → Final validation → Production ready

### Sequential Content Strategy

Since all tasks write to the same file (docs/intro.md):

1. Complete Setup + Foundational (prepare environment)
2. User Story 1: Write core sections (T007-T015)
3. User Story 2: Add structure sections (T016-T020)
4. User Story 3: Enhance with context and navigation (T021-T026)
5. Polish: Final validation and testing (T027-T034)

**Note**: Unlike backend features with separate files, content writing is inherently sequential within a single document. Parallel opportunities exist within each user story for writing different sections.

---

## Notes

- [P] tasks = different sections or validation checks that can be worked on in parallel
- [Story] label maps task to specific user story for traceability
- Each user story adds incremental value: US1 (value/scope) → US2 (structure/planning) → US3 (motivation/navigation)
- Manual testing with target readers (5-10 people) is critical for SC-005 and SC-006
- Docusaurus hot-reload (npm start) allows real-time preview during writing
- All content follows templates and tone guidelines from quickstart.md
- Commit after completing each user story phase for incremental progress tracking
- Avoid: academic tone, passive voice, undefined jargon, missing diagrams, skipping accessibility checks
