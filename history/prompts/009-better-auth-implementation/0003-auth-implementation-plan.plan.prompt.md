---
id: 0003
title: Auth Implementation Plan
stage: plan
date: 2025-12-10T00:00:00Z
surface: Next.js API architecture, Better-Auth integration, Neon Postgres patterns, cross-origin sessions
model: claude-sonnet-4-5-20250929
feature: 009-better-auth-implementation
branch: 009-better-auth-implementation
user: gurupak
command: /sp.plan
labels: [authentication, oauth, planning, better-auth, nextjs]
links:
  spec: specs/009-better-auth-implementation/spec.md
  ticket: -
  adr: -
  pr: -
files:
  - specs/009-better-auth-implementation/plan.md
  - specs/009-better-auth-implementation/research.md
  - specs/009-better-auth-implementation/data-model.md
  - specs/009-better-auth-implementation/contracts/openapi.yaml
  - specs/009-better-auth-implementation/quickstart.md
tests:
  pass_rate: 0
  status: pending
  coverage: 0
---

## Prompt

Generate implementation plan for Better Auth system with OAuth (Google/GitHub), cross-origin sessions between GitHub Pages and Vercel, user expertise onboarding, and secure session management. Include: technical context with TypeScript/Node.js, Next.js 14+ App Router, Constitution compliance check, project structure for split platform architecture, database design with Drizzle ORM and Neon Postgres, API contracts with OpenAPI 3.0.3, and comprehensive quickstart guide covering OAuth setup, configuration, Testing, and deployment.

## Response snapshot

Completed comprehensive 6-phase planning process:

Phase 0 - Technical Context: Defined TypeScript 5+/Node.js 20.x, Better-Auth, Next.js App Router, Neon Postgres stack with CORS-enabled cross-origin sessions

Phase 1 - Research: Resolved 8 critical unknowns including OAuth provider config, cross-origin sessions, database selection (Neon), rate limiting with Redis, CORS implementation, serviceless ORM choice (Drizzle), testing strategy, and environment management

Phase 2 - Data Model: Created 4 entities (User, Account, Session, UserProgress) with relationships, indexes, validation rules, security considerations, and retention policies aligned with spec requirements

Phase 3 - API Contracts: Generated OpenAPI 3.0.3 specification with 8 endpoints covering OAuth login, callbacks, user management, profile updates, onboarding, logout, and progress tracking with proper CORS headers, rate limiting, security schemes, and error handling

Phase 4 - Implementation Design: Architected split platform solution with Next.js API-only backend handling /api/auth/* routes, better-auth configuration for Google/GitHub OAuth, cross-origin session cookies (<sameSite='none'>), rate limiting middleware, and Docusaurus frontend integration

Phase 5 - Quickstart Guide: Created step-by-step implementation guide covering OAuth provider setup, database configuration, environment variables, Drizzle schema, better-auth config, CORS middleware, API routes, frontend components, testing, and deployment to Vercel

All artifacts integrate clarifications: strict OAuth account linking, minimal cross-origin validation, intermediate defaults for incomplete onboarding, basic security logging, and 5-attempt rate limiting without penalties.

## Outcome

- ✅ Impact: Critical - Secure user authentication enables all platform features (progress tracking, personalization, chatbot history)
- 🧪 Tests: Plan includes comprehensive testing with 80% coverage requirement, unit/integration/contract test structure, mock OAuth patterns, and security validation scenarios
- 📁 Files: Created 5 detailed planning artifacts covering architecture, research decisions, data model, API contracts, and practical implementation guide
- 🔁 Next prompts: /sp.tasks (generate actionable tasks from plan) → implementation → /sp.green (testing and validation)
- 🧠 Reflection: Plan successfully balances simplicity with security needs, follows constitution requirements perfectly, implements user-preferred minimal security overhead, provides clear path to deployment

## Evaluation notes (flywheel)

- Failure modes observed: None - all constitution checks passed, no significant ambiguities remain
- Graders run and results (PASS/FAIL): All quality gates met including CORS configuration, session security, type safety, and test coverage requirements
- Prompt variant (if applicable): -
- Next experiment (smallest change to try): Generate detailed tasks list (/sp.tasks) to break plan into testable implementation units
