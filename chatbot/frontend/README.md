# RAG Chatbot Frontend

React + TypeScript frontend for the Interactive Book Learning RAG Chatbot.

## Tech Stack

- **React 18** - UI library
- **TypeScript** - Type safety
- **Vite** - Fast build tool and dev server
- **CSS3** - Styling

## Project Structure

```
chatbot/frontend/
├── src/
│   ├── components/
│   │   ├── ChatWidget.tsx      # Main chat interface
│   │   └── ChatWidget.css      # Chat widget styles
│   ├── App.tsx                 # Main application component
│   ├── App.css                 # Application styles
│   ├── main.tsx                # Application entry point
│   └── index.css               # Global styles
├── index.html                  # HTML template
├── package.json                # Dependencies
├── tsconfig.json               # TypeScript config
└── vite.config.ts              # Vite configuration
```

## Setup

### 1. Install Dependencies

```bash
cd chatbot/frontend
npm install
```

### 2. Configure Backend URL

The frontend is configured to connect to the backend at `http://localhost:8000`. If your backend runs on a different port, update the URLs in:
- `src/App.tsx`
- `src/components/ChatWidget.tsx`
- `vite.config.ts` (proxy configuration)

### 3. Run Development Server

```bash
npm run dev
```

The application will be available at http://localhost:5173

## Features

### Chat Interface

- **Real-time messaging** - Send questions and receive AI-generated responses
- **Conversation history** - View all messages in the current session
- **Expertise level selector** - Choose between beginner, intermediate, or advanced
- **Selected text support** - Ask questions about specific text passages
- **Typing indicators** - Visual feedback while AI is generating response
- **Auto-scroll** - Messages automatically scroll into view

### User Experience

- Responsive design for desktop and mobile
- Clean, modern UI with gradient backgrounds
- Loading states and error handling
- Keyboard shortcuts (Enter to send)
- Session management with automatic creation

## Usage

### Starting a Conversation

1. The app automatically creates a session when it loads
2. Select your expertise level (beginner/intermediate/advanced)
3. Type your question in the input field
4. Press Enter or click the send button

### Asking About Selected Text

1. Paste selected text in the "Paste selected text here" field
2. Type your question about the text
3. Send your message
4. The AI will answer based on the selected text context

### Session Management

- Sessions are created automatically on app load
- Each session has a unique ID
- Sessions expire after 15 minutes of inactivity
- Conversation history is preserved within the session

## API Integration

The frontend communicates with the backend via REST API:

### Endpoints Used

- `POST /api/v1/sessions` - Create a new session
- `GET /api/v1/sessions/{session_id}` - Load session history
- `POST /api/v1/chat` - Send a message
- `POST /api/v1/users/preferences` - Update user preferences

### Authentication

The app sends a `X-User-ID` header with each authenticated request. In production, this should be replaced with actual user authentication from Better Auth.

## Building for Production

```bash
# Build the application
npm run build

# Preview the production build
npm run preview
```

The built files will be in the `dist/` directory.

## Development

### Code Formatting

```bash
npm run lint
```

### Type Checking

TypeScript is configured with strict mode enabled. Run type checking:

```bash
npx tsc --noEmit
```

## Customization

### Styling

Styles are organized in CSS modules:
- `App.css` - Main application layout
- `ChatWidget.css` - Chat interface styling
- `index.css` - Global styles

### Configuration

Key configuration files:
- `vite.config.ts` - Build and dev server settings
- `tsconfig.json` - TypeScript compiler options
- `package.json` - Dependencies and scripts

## Troubleshooting

### Backend Connection Issues

If the frontend can't connect to the backend:
1. Ensure the backend is running on port 8000
2. Check CORS settings in backend (`app/main.py`)
3. Verify proxy configuration in `vite.config.ts`

### Session Creation Fails

If session creation fails:
1. Check backend logs for errors
2. Verify database connection
3. Ensure all backend services are initialized

## Future Enhancements

- Integration with Better Auth for real user authentication
- Rich text rendering for AI responses (markdown support)
- Code syntax highlighting in responses
- File upload for document context
- Voice input support
- Export conversation history

## License

MIT
