# Technical Research: Better Auth Implementation

## Research Questions Addressed

### 1. OAuth Provider Configuration

**Question**: How to configure OAuth providers for better-auth with cross-origin authentication?

**Decision**: Use better-auth's built-in Google and GitHub providers with custom callbacks configured for cross-origin support.

**Rationale**: Better-auth natively supports these providers and provides secure OAuth 2.0 flow implementation. Cross-origin callbacks will be handled through /api/auth/callback/[provider] routes.

**Implementation Details**:
- Google OAuth requires CLIENT_ID and CLIENT_SECRET from Google Cloud Console
- GitHub OAuth requires GITHUB_ID and GITHUB_SECRET from GitHub App settings
- Redirect URI must match: https://<frontend-domain>/api/auth/callback/[provider]

### 2. Cross-Origin Session Management

**Question**: How to maintain secure sessions between GitHub Pages (frontend) and Vercel (API)?

**Decision**: Use better-auth's session configuration with `sameSite: 'none'`, `secure: true`, and `httpOnly: true` cookies.

**Rationale**: This allows cross-origin authentication while maintaining security. The session token can be shared between domains.

**Implementation Details**:
- Session cookies must have explicit domain configuration
- CORS headers must allow credentials from GitHub Pages domain
- CSRF protection must be disabled for cross-origin requests

### 3. Database Selection

**Question**: Which database solution to use for serverless authentication?

**Decision**: Neon Postgres with serverless connection pooling.

**Rationale**: Neon provides serverless PostgreSQL with automatic scaling, perfect for Vercel's serverless functions. It integrates well with modern ORMs and maintains ACID compliance for auth data.

**Implementation Details**:
- Use native Postgres time data types for session management
- Implement connection pooling with pg or @vercel/postgres
- Database URL stored in Vercel environment variables

### 4. Rate Limiting Strategy

**Question**: How to implement rate limiting for authentication endpoints?

**Decision**: Use express-rate-limiter with Redis adapter for distributed rate limiting across Vercel regions.

**Rationale**: Redis provides consistent rate limiting across serverless function invocations. The 5 attempts per 15 minutes per IP strikes a balance between security and user experience.

**Implementation Details**:
- Store rate limit data in Redis (Upstash Redis on Vercel)
- Key format: `auth:rate:ip:<ip_address>`
- Window: 15 minutes, limit: 5 attempts, skip successful logins

### 5. ORM/Query Builder Selection

**Question**: Which database abstraction to use with better-auth and Neon?

**Decision**: Use Drizzle ORM for type-safe queries and schema management.

**Rationale**: Drizzle provides excellent TypeScript support, is optimized for serverless environments, and works seamlessly with Neon. It also supports better-auth adapters.

**Implementation Details**:
- Define schema with Drizzle's schema API
- Generate migrations with Drizzle Kit
- Use prepared statements for security

### 6. Cross-Origin Resource Sharing (CORS)

**Question**: How to properly configure CORS for cross-origin authentication?

**Decision**: Use Next.js middleware to set CORS headers with credentials allowed.

**Rationale**: Manual CORS configuration allows fine-grained control over cross-origin requests for the specific use case of authentication.

**Implementation Details**:
- Allow from: https://*.github.io/ (GitHub Pages pattern)
- Allow credentials: true
- Allow methods: GET, POST, OPTIONS, DELETE
- Allow headers: Authorization, Content-Type, X-Requested-With

### 7. Testing Strategy for Auth

**Question**: How to test authentication without real OAuth providers?

**Decision**: Use mock OAuth providers for unit testing, integration tests with OAuth flow simulation.

**Rationale**: Mock providers allow deterministic testing while integration tests verify the full OAuth flow works correctly.

**Implementation Details**:
- Create mock OAuth endpoints in test environment
- Use better-auth's testing utilities
- Verify token creation, session management, and cross-origin behavior

### 8. Environment Variable Management

**Question**: How to securely manage OAuth secrets across environments?

**Decision**: Use Vercel's environment variables with development, preview, and production scopes.

**Rationale**: Vercel provides secure environment variable management with automatic injection at build time. Different scopes allow safe testing.

**Implementation Details**:
- Development: Local .env files with Next.js loader
- Preview: Vercel environment variables for branch deployments
- Production: Production-only environment variables

## Research Conclusions

This implementation leverages:
1. **Better-auth** for battle-tested authentication with OAuth providers
2. **Neon Postgres** for serverless database scaling
3. **Scoped CORS** for cross-origin authentication
4. **Redis rate limiting** for consistent protection across regions
5. **Drizzle ORM** for type-safe database operations

The architecture follows Next.js App Router patterns with API-only backend, maintaining security through proper cookie configuration and rate limiting while enabling the cross-origin requirements of the split platform.,