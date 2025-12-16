---
id: 0001
title: Better Auth Specification
stage: spec
date: 2025-12-10T00:00:00Z
surface: Authentication UX, OAuth integration pain points, cross-origin session management
model: claude-sonnet-4-5-20250929
feature: 009-better-auth-implementation
branch: 009-better-auth-implementation
user: gurupak
command: /sp.specify
labels: [authentication, oauth, better-auth, specification]
links:
  spec: specs/009-better-auth-implementation/spec.md
  ticket: -
  adr: -
  pr: -
files:
  - specs/009-better-auth-implementation/spec.md
  - specs/009-better-auth-implementation/checklists/requirements.md
tests:
  pass_rate: 0
  status: pending
  coverage: 0
---

## Prompt

Create a comprehensive Better Auth implementation for the Physical AI textbook platform with the following requirements:

Core Authentication System:
- Implement Better-Auth with Google and GitHub OAuth providers
- Set up Next.js API-only backend on Vercel (App Router pattern)
- Configure secure cross-origin sessions between GitHub Pages (frontend) and Vercel API
- Session management with httpOnly, secure cookies with sameSite: 'none'
- Session expiration: 30 days inactive, 90 days absolute

User Onboarding Flow:
- Capture user expertise levels during registration:
  - Python programming level (beginner/intermediate/advanced)
  - ROS 2 experience (none/ROS 1/ROS 2)
  - Hardware access (none simulation/physical robot)
  - Learning goals (career/student/hobbyist/research)

API Structure:
- BASE_URL/api/auth/* for all authentication endpoints
- Implement proper CORS configuration for cross-origin
- Include rate limiting for authentication attempts
- Log all security events for audit trail

Integration Requirements:
- Frontend Docusaurus uses BrowserOnly component for auth widgets
- ChatKit integration requires user context for personalized responses
- Progress tracking API requires authenticated endpoints
- Cross-origin session handling via fetch() with credentials: 'include'

Database Schema:
- Users table with profile metadata
- Sessions table for secure token management
- User_progress table for tracking reading progress
- Accounts table for linking OAuth providers

Testing Requirements:
- All authentication endpoints must have 80% test coverage
- Cross-origin integration tests
- OAuth flow test with mock providers
- Session security validation tests

The authentication system must follow the constitution's split platform architecture (GitHub Pages + Vercel) pattern and integrate seamlessly with the existing Docusaurus frontend.

## Response snapshot

Created comprehensive specification for Better Auth implementation with 4 prioritized user stories (registration, login, cross-origin sessions, password reset), 12 functional requirements covering OAuth integration, secure session management, rate limiting, and audit logging. Defined 8 measurable success criteria including 2-minute registration time, 95% first-attempt success rate, and 99.9% cross-origin API call success rate. Identified key entities (User, Session, Account, User_Progress) and critical edge cases including OAuth outages and session hijacking prevention.

## Outcome

- ✅ Impact: High - Authentication enables all personalized features including progress tracking, chatbot history, and content personalization
- 🧪 Tests: Specification includes testable acceptance scenarios for each user story
- 📁 Files: Created spec.md with user scenarios, requirements, and success criteria; created requirements checklist for validation
- 🔁 Next prompts: /sp.clarify (if needed) → /sp.plan → /sp.tasks → implementation
- 🧠 Reflection: No [NEEDS CLARIFICATION] markers needed - all requirements were well-defined in the original prompt following constitution patterns

## Evaluation notes (flywheel)

- Failure modes observed: None - spec validated successfully
- Graders run and results (PASS/FAIL): All checklist items passed
- Prompt variant (if applicable): -
- Next experiment (smallest change to try): Run /sp.plan to create implementation architecture
