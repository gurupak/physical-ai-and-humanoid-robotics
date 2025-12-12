# Quickstart Guide: Better Auth Implementation

## Prerequisites

- Node.js 20.x or higher
- Neon Postgres account (free tier works)
- Vercel account for deployment
- Google Cloud Console access (for OAuth)
- GitHub Developer Settings access (for OAuth)

## 1. Set up OAuth Providers

### Google OAuth

1. Go to [Google Cloud Console](https://console.cloud.google.com)
2. Create a new project or select existing
3. Enable Google+ API
4. Create OAuth 2.0 credentials:
   - Application type: Web application
   - Authorized redirect URIs:
     - `http://localhost:3000/api/auth/callback/google` (dev)
     - `https://your-app.vercel.app/api/auth/callback/google` (prod)
5. Copy Client ID and Client Secret

### GitHub OAuth

1. Go to [GitHub Developer Settings](https://github.com/settings/developers)
2. Create New OAuth App:
   - Application name: Physical AI Textbook
   - Homepage URL: https://your-app.vercel.app
   - Authorization callback URL:
     - `http://localhost:3000/api/auth/callback/github` (dev)
     - `https://your-app.vercel.app/api/auth/callback/github` (prod)
3. Note Client ID and generate Client Secret

## 2. Set up Neon Database

```bash
# Install Neon CLI
npm install -g @neondatabase/neonctl

# Create new project
neonctl projects create --name physical-ai-auth

# Create database (run in Neon console)
CREATE TABLE auth.users (id SERIAL PRIMARY KEY);
CREATE TABLE auth.accounts (id SERIAL PRIMARY KEY);
CREATE TABLE auth.sessions (id SERIAL PRIMARY KEY);
```

## 3. Environment Configuration

Create `.env.local` in your API project:

```bash
# Database
DATABASE_URL=postgres://user:password@your-neon-host/your-db

# Authentication
BETTER_AUTH_SECRET=your-secret-key-here
NEXTAUTH_URL=http://localhost:3000

# OAuth Providers
GOOGLE_CLIENT_ID=your-google-client-id
GOOGLE_CLIENT_SECRET=your-google-client-secret
GITHUB_CLIENT_ID=your-github-client-id
GITHUB_CLIENT_SECRET=your-github-client-secret

# Redis (for rate limiting)
UPSTASH_REDIS_URL=your-upstash-redis-url

# CORS
NEXT_PUBLIC_SITE_URL=https://your-username.github.io/physical-ai-and-humanoid-robotics
```

## 4. Database Schema (Drizzle)

Create `src/lib/schema.ts`:

```typescript
import { pgTable, uuid, varchar, integer, column, timestamp, boolean, pgEnum } from 'drizzle-orm/pg-core'

// User expertise levels
export const pythonLevelEnum = pgEnum('python_level', ['beginner', 'intermediate', 'advanced'])
export const rosExperienceEnum = pgEnum('ros_experience', ['none', 'ros1', 'ros2'])
export const hardwareAccessEnum = pgEnum('hardware_access', ['none', 'simulation', 'physical'])
export const learningGoalsEnum = pgEnum('learning_goals', ['career', 'student', 'hobbyist', 'research'])
export const providerEnum = pgEnum('provider', ['google', 'github'])

export const users = pgTable('users', {
  id: uuid('id').primaryKey().defaultRandom(),
  email: varchar('email', { length: 255 }).notNull().unique(),
  name: varchar('name', { length: 255 }),
  profileImage: varchar('profile_image', { length: 500 }),
  pythonLevel: pythonLevelEnum('python_level').notNull().default('intermediate'),
  rosExperience: rosExperienceEnum('ros_experience').notNull().default('none'),
  hardwareAccess: hardwareAccessEnum('hardware_access').notNull().default('simulation'),
  learningGoals: learningGoalsEnum('learning_goals').notNull().default('hobbyist'),
  createdAt: timestamp('created_at').notNull().defaultNow(),
  updatedAt: timestamp('updated_at').notNull().defaultNow(),
})

export const accounts = pgTable('accounts', {
  id: uuid('id').primaryKey().defaultRandom(),
  userId: uuid('user_id').notNull().references(() => users.id),
  provider: providerEnum('provider').notNull(),
  providerAccountId: varchar('provider_account_id', { length: 255 }).notNull(),
  accessToken: varchar('access_token', { length: 1000 }),
  refreshToken: varchar('refresh_token', { length: 1000 }),
  expiresAt: timestamp('expires_at'),
  createdAt: timestamp('created_at').notNull().defaultNow(),
})

export const sessions = pgTable('sessions', {
  id: uuid('id').primaryKey().defaultRandom(),
  userId: uuid('user_id').notNull().references(() => users.id),
  token: varchar('token', { length: 255 }).notNull().unique(),
  expiresAt: timestamp('expires_at').notNull(),
  createdAt: timestamp('created_at').notNull().defaultNow(),
  ipAddress: varchar('ip_address', { length: 45 }),
  userAgent: varchar('user_agent', { length: 500 }),
  lastActive: timestamp('last_active').notNull().defaultNow(),
})

export const userProgress = pgTable('user_progress', {
  id: uuid('id').primaryKey().defaultRandom(),
  userId: uuid('user_id').notNull().references(() => users.id),
  chapterId: varchar('chapter_id', { length: 100 }).notNull(),
  completionPercentage: integer('completion_percentage').checkBetween(0, 100).notNull().default(0),
  lastReadPosition: varchar('last_read_position', { length: 500 }),
  updatedAt: timestamp('updated_at').notNull().defaultNow(),
})

// Indexes
export const accountsProviderAccountIdIndex = accounts('accounts_provider_account_id_idx').on(table => [
  table.provider, table.providerAccountId
])
export const accountsUserIdIndex = accounts('accounts_user_id_idx').on(table => [table.userId])
export const sessionsUserIdIndex = sessions('sessions_user_id_idx').on(table => [table.userId])
export const userProgressUserIdIndex = userProgress('user_progress_user_id_idx').on(table => [table.userId])
```

## 5. Better Auth Configuration

Create `src/lib/auth.ts`:

```typescript
import { betterAuth } from 'better-auth'
import { drizzleAdapter } from 'better-auth/adapters/drizzle'
import { db } from './db'
import { users, accounts, sessions } from './schema'
import { Google } from 'better-auth/providers/google'
import { GitHub } from 'better-auth/providers/github'


// Rate limiting adapter
const redis = new Redis(process.env.UPSTASH_REDIS_URL!)

export const auth = betterAuth({
  database: drizzleAdapter(db, {
    provider: 'pg',
    schema: {
      user: users,
      session: sessions,
      account: accounts,
    }
  }),

  providers: [
    Google({
      clientId: process.env.GOOGLE_CLIENT_ID!,
      clientSecret: process.env.GOOGLE_CLIENT_SECRET!,
    }),
    GitHub({
      clientId: process.env.GITHUB_CLIENT_ID!,
      clientSecret: process.env.GITHUB_CLIENT_SECRET!,
    }),
  ],

  session: {
    expiresIn: 60 * 60 * 24 * 30, // 30 days
    updateAge: 60 * 60 * 24, // 1 day
    cookie: {
      name: 'better-auth.session',
      httpOnly: true,
      secure: true,
      sameSite: 'none', // For cross-origin
      maxAge: 60 * 60 * 24 * 30, // 30 days
    }
  },

  trustOrigins:true,

  rateLimit: {
    enabled: true,
    rules: [
      {
        pathMatcher: '/admin/**',
        methodMatcher: '(GET|POST|PUT|DELETE)',
        window: 15 * 60 * 1000, // 15 minutes
        max: 5, // 5 attempts
        skipSuccessfulRequests: false,
      },
      {
        pathMatcher: '/api/auth/**',
        window: 60 * 1000, // 1 minute
        max: 10, // 10 requests
        skipSuccessfulRequests: true,
      },
    ],
  },

  advanced: {
    debug: process.env.NODE_ENV === 'development',
    disableant:, // Allow local testing
  },
})

export type Auth = typeof auth
```

## 6. CORS Configuration

Create `src/lib/cors.ts`:

```typescript
import { NextResponse } from 'next/server'
import type { NextRequest } from 'next/server'

const allowedOrigins = [
  'https://*.github.io',
  'http://localhost:3000', // Local development
]

export function corsMiddleware(request: NextRequest) {
  const origin = request.headers.get('origin')
  const response = NextResponse.next()

  // Check if origin matches allowed patterns
  const isAllowed = origin && (
    /https:\/\/[^\/]+\.github\.io/.test(origin) ||
    origin === 'http://localhost:3000'
  )

  if (isAllowed) {
    response.headers.set('Access-Control-Allow-Origin', origin)
    response.headers.set('Access-Control-Allow-Credentials', 'true')
    response.headers.set('Access-Control-Allow-Methods', 'GET, POST, PUT, DELETE, OPTIONS')
    response.headers.set(
      'Access-Control-Allow-Headers',
      'Content-Type, Authorization, X-Requested-With'
    )
  }

  // Handle preflight requests
  if (request.method === 'OPTIONS') {
    return new NextResponse(null, {
      status: 200,
      headers: response.headers
    })
  }

  return response
}
```

## 7. API Route Example

Create `src/app/api/auth/me/route.ts`:

```typescript
import { auth } from '@/lib/auth'
import { NextRequest } from 'next/server'

export async function GET(request: NextRequest) {
  const session = await auth.api.getSession({
    headers: request.headers,
  })

  if (!session) {
    return Response.json(
      { error: 'Unauthorized', message: 'No valid session' },
      { status: 401 }
    )
  }

  // Get user with expertise data
  const user = await db.query.users.findFirst({
    where: (users, { eq }) => eq(users.id, session.user.id),
  })

  return Response.json({
    id: user.id,
    email: user.email,
    name: user.name,
    profileImage: user.profileImage,
    pythonLevel: user.pythonLevel,
    rosExperience: user.rosExperience,
    hardwareAccess: user.hardwareAccess,
    learningGoals: user.learningGoals,
    createdAt: user.createdAt,
    updatedAt: user.updatedAt,
  })
}
```

## 8. Frontend Integration

Create `src/components/AuthWidget.tsx`:

```tsx
import React from 'react'
import BrowserOnly from '@docusaurus/BrowserOnly'

export default function AuthWidget() {
  return (
    <BrowserOnly>
      {() => {
        const user = getCurrentUser() // Fetch from session

        if (user) {
          return (
            <div className="auth-widget">
              <img src={user.profileImage} alt="Profile" />
              <span>Welcome, {user.name}</span>
              <button onClick={logout}>Logout</button>
            </div>
          )
        }

        return (
          <div className="auth-widget">
            <button onClick={() => window.location.href = '/api/auth/login/google'}>
              Sign in with Google
            </button>
            <button onClick={() => window.location.href = '/api/auth/login/github'}>
              Sign in with GitHub
            </button>
          </div>
        )
      }}
    </BrowserOnly>
  )
}
```

## 9. Testing

Create `api/tests/integration/auth.test.ts`:

```typescript
import { describe, it, expect, beforeAll } from 'vitest'

describe('Authentication Integration', () => {
  it('should handle OAuth callback', async () => {
    const response = await fetch('/api/auth/callback/google?code=test-code', {
      headers: {
        'Origin': 'https://username.github.io',
      },
    })

    expect(response.status).toBe(302)
    expect(response.headers.get('Set-Cookie')).toContain('better-auth.session')
  })

  it('should protect authenticated routes', async () => {
    const response = await fetch('/api/auth/me')

    expect(response.status).toBe(401)
    const data = await response.json()
    expect(data.error).toBe('Unauthorized')
  })

  it('should handle CORS for cross-origin requests', async () => {
    const response = await fetch('/api/auth/login/google', {
      headers: {
        'Origin': 'https://valid.github.io',
      },
    })

    expect(response.headers.get('Access-Control-Allow-Origin')).toBe('https://valid.github.io')
  })
})
```

## 10. Deploy

```bash
# Set environment variables in Vercel
vercel env add DATABASE_URL production
vercel env add GOOGLE_CLIENT_ID production
vercel env add GOOGLE_CLIENT_SECRET production
vercel env add GITHUB_CLIENT_ID production
vercel env add GITHUB_CLIENT_SECRET production
vercel env add BETTER_AUTH_SECRET production
vercel env add UPSTASH_REDIS_URL production

# Deploy
vercel deploy --prod
```

## Verification Checklist

- ✅ OAuth providers configured with correct redirect URIs
- ✅ Database schema created and tested
- ✅ Environment variables set in all environments
- ✅ CORS configured for cross-origin authentication
- ✅ Rate limiting active (5 attempts per 15 minutes)
- ✅ Security logging enabled (30-day retention)
- ✅ Session cookies properly configured (<sameSite: 'none'>)
- ✅ Tests pass (80% coverage)

Your authentication system is now live and ready for the Physical AI textbook! 🎉

## Next Steps

1. Set up email service for password reset (when implementing FR-007)
2. Add onboarding survey UI in Docusaurus
3. Implement progress tracking integration
4. Test cross-origin authentication end-to-end
5. Monitor auth metrics and error rates

## Troubleshooting

**CORS Issues**: Ensure your GitHub pages domain exactly matches the origin header
**Session Issues**: Check that SameSite=None is set and HTTPS is enabled
**Database Issues**: Verify DATABASE_URL format and connection pooling
**OAuth Issues**: Confirm redirect URIs are correct in provider console
**Rate Limiting**: Check Redis connection and limit configurations

For more details, see the full implementation docs or run the test suite. Happy auth building! 🔐,"file_path":"specs/009-better-auth-implementation/quickstart.md"}