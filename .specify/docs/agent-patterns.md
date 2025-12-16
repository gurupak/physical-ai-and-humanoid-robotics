# OpenAI Agents JS SDK - Code Patterns

Reference implementation patterns for OpenAI Agents JS SDK. Only load when implementing agent features.

## Agent Creation

```typescript
import { Agent, tool } from '@openai/agents';
import { z } from 'zod';

const myAgent = new Agent({
  name: 'Book Q&A Agent',
  instructions: 'You answer questions about robotics textbook content...',
  tools: [searchBookContentTool],
  handoffs: [selectedTextAgent], // Optional: agents to hand off to
  model: 'gpt-4o', // Or 'gpt-4o-mini' for cost savings
});
```

## Tool Definition

```typescript
const searchBookContentTool = tool({
  name: 'search_book_content',
  description: 'Search the robotics textbook for relevant content',
  parameters: z.object({
    query: z.string().describe('Search query for textbook content'),
    chapter: z.string().optional().describe('Limit search to specific chapter'),
  }),
  execute: async ({ query, chapter }) => {
    // Hybrid search: Qdrant vector + keyword
    const results = await hybridSearch(query, chapter);
    return { results, citations: extractCitations(results) };
  },
});
```

## Handoff Patterns

```typescript
// Triage agent with handoffs
const triageAgent = new Agent({
  name: 'Triage Agent',
  instructions: 'Route user queries to appropriate specialist...',
  handoffs: [bookQAAgent, selectedTextAgent],
  model: 'gpt-4o-mini',
});

// IMPORTANT: No circular handoffs (A → B → A creates loops)
// Each agent should have clear specialization
```

## Session Management

```typescript
import { OpenAIConversationsSession } from '@openai/agents';

const session = new OpenAIConversationsSession({
  store: neonPostgresStore, // Custom store implementation
  sessionId: userId,
  expiresIn: 90 * 24 * 60 * 60, // 90 days
});

const response = await agent.run({
  session,
  message: userMessage,
  streamingMode: 'full', // Enable streaming
});
```

## Built-in Tools

```typescript
import { webSearchTool } from '@openai/agents';

// Use sparingly for supplementing textbook with current news
const newsAgent = new Agent({
  name: 'News Agent',
  instructions: 'Supplement textbook with current robotics developments...',
  tools: [webSearchTool()],
  model: 'gpt-4o',
});
```
