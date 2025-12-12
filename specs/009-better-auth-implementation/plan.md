# Implementation Plan: Better Auth Implementation with OAuth

**Branch**: `[009-better-auth-implementation]` | **Date**: 2025-12-10 | **Spec**: [specs/009-better-auth-implementation/spec.md](spec.md)
**Input**: Feature specification from `/specs/009-better-auth-implementation/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implement Better-Auth with OAuth (Google, GitHub) for the Physical AI textbook platform. Create a Next.js API-only backend on Vercel that handles authentication, secure cross-origin sessions between GitHub Pages frontend and API backend, and user onboarding with expertise capture. The system uses `sameSite: 'none'` cookies for cross-origin compatibility and maintains sessions for 30 days inactive, 90 days maximum. User profiles capture Python level, ROS experience, hardware access, and learning goals to enable personalized content delivery.

## Technical Context

**Language/Version**: TypeScript 5+, Node.js 20.x
**Primary Dependencies**: Better-Auth, Next.js 14+ App Router, @openai/agents, zod, @neondatabase/serverless
**Storage**: Neon Postgres (PostgreSQL) serverless database
**Testing**: Jest/Vitest with 80% test coverage requirement for API routes
**Target Platform**: Next.js App Router API-only on Vercel
**Project Type**: Web - Split platform (static frontend + API backend)
**Performance Goals**: Support 1000 concurrent authenticated sessions without degradation
**Constraints**: Cross-origin authentication between GitHub Pages and Vercel, rate limiting 5 attempts per 15min/IP
**Scale/Scope**: Education platform, 3-10 chapters, OAuth only (no password auth initially)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Technology Stack Compliance**: ✅ PASSED
- Better-Auth specified for authentication (XI:211)
- Next.js 14+ App Router for API backend (V:88-90)
- Neon Postgres serverless database
- Cross-origin sessions required (V:216)

**Architecture Compliance**: ✅ PASSED
- Split platform architecture (III:44) - GitHub Pages + Vercel
- API routes only (`/api/auth/*` per V:98 and XI:215)
- Environment variables via Vercel dashboard (V:55)

**Standards Compliance**: ✅ PASSED
- TypeScript strict mode (V:93)
- Functional components with hooks
- Tailwind CSS for styling
- Testing requirements (80% coverage for API routes)

**Performance & Quality Gates**: ✅ PASSED
- WCAG AA accessibility (XIV:261)
- Core Web Vitals thresholds (XIV:270)
- Rate limiting implemented (spec FR-008)

## Project Structure

### Documentation (this feature)

```text
specs/009-better-auth-implementation/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Option 2: Web application - split platform architecture
api/
├── app/
│   ├── api/
│   │   ├── auth/
│   │   │   ├── [...nextauth]/route.ts
│   │   │   ├── callback/[provider]/route.ts
│   │   │   ├── profile/route.ts
│   │   │   ├── onboard/route.ts
│   │   │   └── logout/route.ts
│   │   └── progress/route.ts
│   ├── lib/
│   │   ├── auth.ts         # Better-auth configuration
│   │   ├── db.ts          # Neon connection
│   │   ├── cors.ts        # CORS configuration
│   │   └── types.ts        # TypeScript types
│   └── middleware.ts      # Rate limiting, CORS
│   └── config.ts          # Environment validation
│
└── tests/
    ├── unit/
    ├── integration/
    │   ├── auth.test.ts
    │   ├── oauth.test.ts
    │   └── cors.test.ts
    └── contract/
    │   └── auth-contracts.ts

# Frontend integration in existing Docusaurus
src/
└── components/
    ├── AuthWidget.tsx
    ├── OnboardingModal.tsx
    └── BrowserOnly/          # Auth components wrapper
```

**Structure Decision**: Web application with split platform architecture (Option 2) aligns with constitution requirements III-V. API-only Next.js backend on Vercel serving the GitHub Pages frontend. Separate tests for unit, integration, and contract testing.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

**No violations detected** - All technology choices and architecture patterns conform to constitution guidelines.