# RAG Chatbot Backend

Backend API for the Interactive Book Learning RAG Chatbot using FastAPI, OpenAI Agents SDK, and Qdrant.

## Tech Stack

- **Python 3.11+**
- **FastAPI** - Modern web framework for building APIs
- **OpenAI Agents SDK** - Multi-agent orchestration
- **Qdrant Cloud** - Vector database for semantic search
- **Neon Postgres** - Serverless Postgres for user data
- **uv** - Fast Python package manager

## Project Structure

```
chatbot/backend/
├── app/
│   ├── agents/          # AI agents (triage, book_qa, selected_text)
│   ├── api/             # FastAPI routes and endpoints
│   ├── core/            # Configuration and settings
│   ├── db/              # Database clients (Qdrant, Postgres)
│   ├── models/          # Pydantic models and schemas
│   ├── services/        # Business logic services
│   ├── utils/           # Utility functions
│   └── main.py          # FastAPI application
├── scripts/             # Data ingestion and utility scripts
├── tests/               # Unit and integration tests
├── pyproject.toml       # Python dependencies
└── run.py               # Development server
```

## Setup

### 1. Install Dependencies

```bash
cd chatbot/backend
uv sync
```

### 2. Configure Environment Variables

Create a `.env` file in the project root (not in chatbot/backend):

```env
# OpenAI API Configuration
OPENAI_API_KEY=sk-...

# Qdrant Vector Database Configuration
QDRANT_URL=https://your-cluster-name.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key

# Neon Postgres Database
DATABASE_URL=postgresql://user:password@host:5432/dbname

# Application Configuration
NODE_ENV=development
PORT=8000
```

### 3. Initialize Database

The application will automatically create required tables on startup.

### 4. Ingest Book Content

Load book content into Qdrant vector database:

```bash
# From project root
uv run chatbot/backend/scripts/ingest_book_content.py

# Or specify a custom docs directory
uv run chatbot/backend/scripts/ingest_book_content.py /path/to/docs
```

## Running the Server

### Development Mode

```bash
# Using uv
uv run chatbot/backend/run.py

# Or using uvicorn directly
uv run uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

The API will be available at:
- **API**: http://localhost:8000
- **Docs**: http://localhost:8000/docs
- **OpenAPI**: http://localhost:8000/openapi.json

## API Endpoints

### Sessions

- `POST /api/v1/sessions` - Create a new conversation session
- `GET /api/v1/sessions/{session_id}` - Get session details

### Chat

- `POST /api/v1/chat` - Send a message and get a response
  - Header: `X-User-ID` (required)
  - Body: `{ "session_id": "...", "message": "...", "selected_text": "..." }`

### User Preferences

- `POST /api/v1/users/preferences` - Create/update user preferences
- `GET /api/v1/users/preferences` - Get user preferences
- `PATCH /api/v1/users/preferences` - Update user preferences
  - Header: `X-User-ID` (required)

### Health

- `GET /api/v1/health` - Health check endpoint

## Development

### Code Quality

```bash
# Format code
uv run ruff format .

# Lint code
uv run ruff check .

# Type checking
uv run mypy app/
```

### Testing

```bash
# Run tests
uv run pytest

# Run with coverage
uv run pytest --cov=app tests/
```

## Architecture

### Multi-Agent System

1. **Triage Agent** - Routes queries to appropriate specialists
2. **Book Q&A Agent** - Answers general book questions using RAG
3. **Selected Text Agent** - Answers questions about user-selected text

### RAG Pipeline

1. User sends query with optional selected text
2. Query analyzer determines complexity and type
3. For general queries: Generate embeddings → Retrieve chunks from Qdrant
4. Route to appropriate agent (Book Q&A or Selected Text)
5. Agent generates response using OpenAI with context
6. Save conversation to Postgres
7. Return response to user

### Session Management

- 15-minute session timeout
- 10-turn conversation history limit
- Automatic cleanup of expired sessions

## Configuration

All configuration is managed through environment variables and `app/core/config.py`.

Key settings:
- `CHUNK_SIZE`: Default 1000 characters
- `CHUNK_OVERLAP`: Default 200 characters
- `MIN_CHUNKS`: Minimum 3 chunks retrieved
- `MAX_CHUNKS`: Maximum 10 chunks retrieved
- `SESSION_TIMEOUT_MINUTES`: 15 minutes
- `MAX_CONVERSATION_TURNS`: 10 turns

## Troubleshooting

### Database Connection Issues

Ensure your `DATABASE_URL` is correct and the Neon Postgres database is accessible.

### Qdrant Connection Issues

Verify your `QDRANT_URL` and `QDRANT_API_KEY` are correct. Check Qdrant Cloud dashboard.

### OpenAI API Errors

Confirm your `OPENAI_API_KEY` is valid and has sufficient credits.

## License

MIT
