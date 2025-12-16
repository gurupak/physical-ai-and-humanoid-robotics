# ChatKit Integration in Docusaurus

Reference implementation for embedding ChatKit in Docusaurus. Only load when implementing chatbot UI.

## Component Structure

```tsx
// src/components/ChatbotWidget.tsx
import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';

export default function ChatbotWidget() {
  return (
    <BrowserOnly fallback={<div>Loading chatbot...</div>}>
      {() => {
        const { ChatKit } = require('@openai/chatkit'); // Dynamic import
        return (
          <ChatKit
            apiEndpoint="https://your-vercel-api.vercel.app/api/chat"
            authEndpoint="https://your-vercel-api.vercel.app/api/auth/session"
            onError={(error) => console.error('ChatKit error:', error)}
            enableStreaming={true}
            placeholder="Ask about robotics concepts..."
          />
        );
      }}
    </BrowserOnly>
  );
}
```

## Selected Text Feature

```tsx
// src/components/ChatbotWidget.tsx
import React, { useState, useEffect } from 'react';

export default function ChatbotWidget() {
  const [selectedText, setSelectedText] = useState('');

  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();
      if (text && text.length > 0) {
        setSelectedText(text);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => document.removeEventListener('mouseup', handleSelection);
  }, []);

  return (
    <BrowserOnly>
      {() => {
        const { ChatKit } = require('@openai/chatkit');
        return (
          <>
            {selectedText && (
              <FloatingButton
                text="Ask about this"
                onClick={() => {
                  // Open chatbot with selected text as context
                }}
              />
            )}
            <ChatKit
              apiEndpoint="https://your-api.vercel.app/api/chat"
              initialContext={selectedText}
              // ... other props
            />
          </>
        );
      }}
    </BrowserOnly>
  );
}
```

## API Communication with Credentials

```typescript
// Inside ChatKit or custom fetch
fetch('https://your-api.vercel.app/api/chat', {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  credentials: 'include', // Include cookies for auth
  body: JSON.stringify({ message, sessionId, selectedText }),
});
```

## Citation Display

```tsx
function parseCitations(response: string) {
  // Parse response for citations in format: chapter/section:line-range
  const citationRegex = /\[([^\]]+):(\d+-\d+)\]/g;

  return response.replace(citationRegex, (match, ref, lines) => {
    const [chapter, section] = ref.split('/');
    const url = `/docs/${chapter}#${section}`;
    return `<a href="${url}" class="citation">${match}</a>`;
  });
}
```

## Docusaurus Theme Integration

```typescript
// docusaurus.config.ts
export default {
  themeConfig: {
    // Register custom component
    navbar: {
      items: [
        // ... other items
        {
          type: 'custom-chatbot-widget',
          position: 'right',
        },
      ],
    },
  },
};
```
