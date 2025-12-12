# Feature Specification: Better Auth Implementation with OAuth

**Feature Branch**: `[009-better-auth-implementation]`
**Created**: 2025-12-10
**Status**: Draft
**Input**: User description: "Create a comprehensive Better Auth implementation for the Physical AI textbook platform with OAuth (Google, GitHub), secure cross-origin sessions, user onboarding flow, and Next.js API backend"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - First-Time Visitor Registration (Priority: P1)

A first-time visitor to the Physical AI textbook can quickly register using their Google or GitHub account without creating a new password. After authentication, they complete a brief onboarding survey about their robotics experience level.

**Why this priority**: Registration is the entry point for all personalized features. Without authentication, users cannot access progress tracking, personalized content, or chatbot history.

**Independent Test**: Can be fully tested by clicking "Sign Up" on the website, completing OAuth flow, and verifying account creation in the system.

**Acceptance Scenarios**:

1. **Given** a visitor on the textbook homepage, **When** they click "Sign Up" and choose Google OAuth, **Then** they are redirected to Google authentication and returned to complete registration
2. **Given** a visitor completes Google OAuth successfully, **When** they land on the onboarding survey, **Then** they can select their Python/ROS/hardware experience levels
3. **Given** a user completes the onboarding survey, **When** they submit their preferences, **Then** their profile is created and they are logged into the platform

---

### User Story 2 - Returning User Login (Priority: P1)

Returning users can log in with one click using their previously connected OAuth provider and immediately access their saved progress and personalized features.

**Why this priority**: Frictionless login encourages repeat visits and continued learning. Users should not need to remember passwords or re-enter credentials.

**Independent Test**: Can be tested by visiting the site after account creation, clicking "Sign In", and verifying immediate access to saved progress.

**Acceptance Scenarios**:

1. **Given** a registered user visits the textbook, **When** they click "Sign In" and select their OAuth provider, **Then** they are authenticated without entering credentials
2. **Given** a user successfully logs in, **When** they navigate to any chapter, **Then** their reading progress indicators show correctly
3. **Given** a logged-in user refreshes the page, **When** the page loads, **Then** their session persists and login state remains active

---

### User Story 3 - Cross-Origin Session Management (Priority: P1)

Users can authenticate through the static Docusaurus site (GitHub Pages) and maintain secure sessions with the API backend (Vercel) without encountering CORS issues.

**Why this priority**: The split platform architecture requires secure cross-origin authentication. This is fundamental to the entire system design.

**Independent Test**: Can be tested by authenticating on the static site and verifying API calls from the same browser session are authenticated.

**Acceptance Scenarios**:

1. **Given** a user on the GitHub Pages frontend, **When** they authenticate via OAuth, **Then** the session cookie is properly set for cross-origin API access
2. **Given** an authenticated user, **When** their browser makes API calls from GitHub Pages to Vercel, **Then** the requests include valid session credentials
3. **Given** a user closes and reopens their browser, **When** they return within 30 days, **Then** their session remains valid without re-authentication

---

### User Story 4 - Password Reset Flow (Priority: P2)

Users who signed up with email/password can reset their password if forgotten, receiving a secure reset link via email.

**Why this priority**: Secondary to OAuth but necessary for users unable or unwilling to use OAuth providers.

**Independent Test**: Can be tested by clicking "Forgot Password", entering email, and successfully resetting password via received link.

**Acceptance Scenarios**:

1. **Given** a user clicks "Forgot Password", **When** they enter their registered email, **Then** they receive a password reset email within 5 minutes
2. **Given** a user receives a password reset link, **When** they click it within 1 hour, **Then** they can set a new password successfully
3. **Given** a user sets a new password, **When** they try to log in with it, **Then** authentication succeeds immediately

---

### Edge Cases

- What happens when OAuth provider is temporarily unavailable? (Google/GitHub outage)
- How does system handle users who change their email address on the OAuth provider?
- How are rate limits enforced when multiple failed login attempts occur from the same IP?
- What happens when a user tries to register with an OAuth account already linked to another user?
- How does the system handle sessions when user clears browser cookies?
- What security measures prevent session hijacking in cross-origin scenarios?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST allow users to sign up using Google OAuth without creating a password
- **FR-002**: System MUST allow users to sign up using GitHub OAuth without creating a password
- **FR-003**: System MUST authenticate users across GitHub Pages frontend and Vercel API backend
- **FR-004**: System MUST maintain secure session state for 30 days of inactivity (90 days maximum)
- **FR-005**: System MUST capture user expertise profile during onboarding (Python level, ROS experience, hardware access, learning goals)
- **FR-006**: System MUST allow users to view and update their profile information after registration
- **FR-007**: System MUST provide password reset functionality for email/password accounts
- **FR-008**: System MUST enforce rate limiting on authentication attempts (max 5 failed attempts per 15 minutes per IP)
- **FR-009**: System MUST log all security events (login failures, password resets, suspicious activity)
- **FR-010**: System MUST prevent session hijacking through secure cookie configuration and accept cross-origin requests from any GitHub Pages subdomain
- **FR-011**: System MUST allow users to logout and invalidate their session globally
- **FR-012**: System MUST support multiple OAuth providers linked to single user account

### Key Entities

- **User**: Represents a platform user with profile metadata (id, email, name, profile image, expertise level, learning goals, created_at, updated_at)
- **Session**: Secure authentication token (id, user_id, expires_at, created_at, ip_address, user_agent)
- **Account**: OAuth provider links (id, user_id, provider, provider_account_id, refresh_token, access_token, expires_at)
- **User_Progress**: Learning progress tracking (id, user_id, chapter_id, completion_percentage, last_read_position, updated_at)

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can complete OAuth registration and onboarding in under 2 minutes
- **SC-002**: 95% of users successfully authenticate on first attempt using OAuth
- **SC-003**: Session persistence works across browser restarts for 95% of users
- **SC-004**: Cross-origin API calls succeed for authenticated users 99.9% of the time
- **SC-005**: Password reset requests are delivered within 5 minutes for 99% of requests
- **SC-006**: User profiles are created with complete onboarding data for 100% of new registrations
- **SC-007**: The system handles 1000 concurrent authenticated sessions without performance degradation
- **SC-008**: Authentication-related support requests are reduced to less than 2% of total user base monthly

## Assumptions

- Users have access to Google or GitHub accounts for OAuth
- Email service is available for password reset functionality
- Vercel deployment supports required cookie configuration for cross-origin sessions
- Users complete onboarding survey within 24 hours of OAuth registration
- Rate limiting thresholds balance security with user experience
- Each email address can only be associated with one OAuth provider account (strict linking approach)
- Incomplete onboarding defaults to "intermediate" for all expertise levels

## Clarifications

### Session 2025-12-10

#### Policy Clarification: OAuth Provider Linking
**IMPORTANT**: This clarification OVERRIDES FR-012. The system implements a STRICT linking approach where each email address can only use ONE OAuth provider. Users cannot link multiple OAuth providers to the same account. Attempts to authenticate with a different provider for the same email will be rejected.

- Q: How should the system handle users who register with different OAuth providers at different times? → A: Strict - Each email can only have one OAuth provider; reject attempts to link additional providers
- Q: How strictly should we validate the cross-origin setup for security? → A: Minimal - Allow any GitHub Pages subdomain to make authenticated requests
- Q: How should incomplete onboarding be handled if users skip the survey? → A: Set defaults to "intermediate" for all expertise levels when onboarding is incomplete
- Q: What should be included in security logs and how long retained? → A: Basic logging: timestamp, user ID, event type, IP address (30 days retention)
- Q: What should happen when users exceed rate limits? → A: 5 failed attempts threshold is acceptable without additional lockout