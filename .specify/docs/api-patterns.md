# Next.js API Backend - Code Patterns

Reference implementation patterns for Next.js API routes. Only load when implementing backend features.

## CORS Middleware

```typescript
// middleware.ts
import { NextResponse } from 'next/server';

export function middleware(request: Request) {
  const origin = request.headers.get('origin');
  const allowedOrigins = [
    'https://username.github.io',
    'http://localhost:3000', // for local development
  ];

  if (allowedOrigins.includes(origin)) {
    return NextResponse.next({
      headers: {
        'Access-Control-Allow-Origin': origin,
        'Access-Control-Allow-Credentials': 'true',
        'Access-Control-Allow-Methods': 'GET,POST,OPTIONS',
        'Access-Control-Allow-Headers': 'Content-Type, Authorization',
      },
    });
  }

  return NextResponse.next();
}

export const config = { matcher: '/api/:path*' };
```

## Streaming SSE Response

```typescript
// app/api/chat/route.ts
export async function POST(req: Request) {
  const stream = new TransformStream();
  const writer = stream.writable.getWriter();

  // Agent streaming logic
  const response = await agent.run({
    session,
    message: userMessage,
    streamingMode: 'full',
  });

  // Stream to client
  for await (const chunk of response) {
    await writer.write(new TextEncoder().encode(`data: ${JSON.stringify(chunk)}\n\n`));
  }

  await writer.close();

  return new Response(stream.readable, {
    headers: {
      'Content-Type': 'text/event-stream',
      'Cache-Control': 'no-cache',
      'Connection': 'keep-alive',
    },
  });
}
```

## Request Validation with Zod

```typescript
import { z } from 'zod';

const chatRequestSchema = z.object({
  message: z.string().min(1).max(1000),
  sessionId: z.string().uuid(),
  selectedText: z.string().optional(),
});

export async function POST(req: Request) {
  const body = await req.json();
  const validated = chatRequestSchema.parse(body); // Throws if invalid

  // Use validated.message, validated.sessionId, etc.
}
```

## Environment Variables

```typescript
// Validate env vars at startup
const envSchema = z.object({
  OPENAI_API_KEY: z.string().min(1),
  QDRANT_URL: z.string().url(),
  QDRANT_API_KEY: z.string().min(1),
  DATABASE_URL: z.string().url(),
  BETTER_AUTH_SECRET: z.string().min(32),
});

export const env = envSchema.parse(process.env);
```
