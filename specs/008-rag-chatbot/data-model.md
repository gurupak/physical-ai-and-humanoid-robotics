# Data Model: RAG Chatbot

**Feature**: 008-rag-chatbot  
**Date**: 2025-12-11

## Overview

This document defines the data models for the RAG chatbot feature, including database schemas, API request/response models, and in-memory session structures.

---

## Database Schema (Neon Postgres)

### Existing: `users` table (Better Auth)

Already implemented by Better Auth. Relevant fields:

```sql
TABLE users (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  email VARCHAR(255) UNIQUE NOT NULL,
  name VARCHAR(255),
  image TEXT,
  emailVerified BOOLEAN DEFAULT FALSE,
  createdAt TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updatedAt TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  
  -- Custom field for chatbot
  expertiseLevel VARCHAR(20) DEFAULT 'beginner' CHECK (expertiseLevel IN ('beginner', 'intermediate', 'advanced'))
);
```

**Notes**:
- `expertiseLevel` field already exists (configured per clarifications)
- No additional tables needed for MVP
- Session management handled by Better Auth

---

## Vector Database Schema (Qdrant)

### Collection: `book_content`

**Vector configuration**:
```python
{
  "collection_name": "book_content",
  "vectors": {
    "size": 1536,  # text-embedding-3-small dimension
    "distance": "Cosine"
  }
}
```

**Point structure**:
```python
{
  "id": str,  # Unique chunk ID (e.g., "ch3_sec2_chunk5")
  "vector": List[float],  # 1536-dimensional embedding
  "payload": {
    "chapter": str,        # "Chapter 3"
    "section": str,        # "3.2 Digital Twins"
    "subsection": str,     # "3.2.1 Overview" (optional)
    "content": str,        # Raw text content
    "line_range": str,     # "45-78" (for citations)
    "file_path": str,      # "docs/digital-twin/overview.mdx"
    "chunk_index": int,    # Sequential number within document
    "token_count": int,    # Approximate token count
    "created_at": str      # ISO timestamp of indexing
  }
}
```

**Example point**:
```python
{
  "id": "ch3_sec2_chunk5",
  "vector": [0.123, -0.456, ...],  # 1536 floats
  "payload": {
    "chapter": "Chapter 3",
    "section": "3.2 Digital Twins in Robotics",
    "subsection": "3.2.1 Core Concepts",
    "content": "Digital twins are virtual replicas of physical systems...",
    "line_range": "45-78",
    "file_path": "docs/digital-twin/core-concepts.mdx",
    "chunk_index": 5,
    "token_count": 512,
    "created_at": "2025-12-11T10:30:00Z"
  }
}
```

---

## API Request/Response Models (Pydantic)

### 1. Chat Request

```python
from pydantic import BaseModel, Field
from typing import Optional, List

class ChatRequest(BaseModel):
    """Request model for chat endpoint"""
    query: str = Field(..., min_length=1, max_length=2000, description="User's question")
    selected_text: Optional[str] = Field(None, max_length=10000, description="Selected text from book")
    session_id: str = Field(..., description="Better Auth session ID")
    
    model_config = {
        "json_schema_extra": {
            "examples": [{
                "query": "What is a Vision-Language-Action model?",
                "selected_text": None,
                "session_id": "sess_abc123xyz"
            }]
        }
    }
```

### 2. Chat Response

```python
from typing import List, Dict, Any

class Citation(BaseModel):
    """Citation to book section"""
    chapter: str
    section: str
    line_range: str
    file_path: str
    
class ChatResponse(BaseModel):
    """Response model for chat endpoint"""
    answer: str = Field(..., description="Generated answer")
    citations: List[Citation] = Field(default_factory=list, description="Source citations")
    agent_used: str = Field(..., description="Which agent handled the query")
    confidence: float = Field(..., ge=0.0, le=1.0, description="Confidence score")
    chunks_retrieved: int = Field(..., description="Number of chunks used")
    
    model_config = {
        "json_schema_extra": {
            "examples": [{
                "answer": "A Vision-Language-Action (VLA) model...",
                "citations": [
                    {
                        "chapter": "Chapter 4",
                        "section": "4.1 Introduction to VLA",
                        "line_range": "12-25",
                        "file_path": "docs/vla/introduction.mdx"
                    }
                ],
                "agent_used": "Book Q&A Agent",
                "confidence": 0.92,
                "chunks_retrieved": 5
            }]
        }
    }
```

### 3. Expertise Level Update

```python
from enum import Enum

class ExpertiseLevel(str, Enum):
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"

class ExpertiseUpdateRequest(BaseModel):
    """Request to update user expertise level"""
    session_id: str
    expertise_level: ExpertiseLevel
    
class ExpertiseUpdateResponse(BaseModel):
    """Response after updating expertise"""
    success: bool
    new_level: ExpertiseLevel
```

### 4. Error Response

```python
class ErrorDetail(BaseModel):
    """Detailed error information"""
    error_code: str
    message: str
    service: Optional[str] = None  # "openai", "qdrant", "database"
    retry_after: Optional[int] = None  # Seconds to wait before retry
    
class ErrorResponse(BaseModel):
    """Standardized error response"""
    error: ErrorDetail
    
    model_config = {
        "json_schema_extra": {
            "examples": [{
                "error": {
                    "error_code": "RATE_LIMIT_EXCEEDED",
                    "message": "Too many requests. Please wait before retrying.",
                    "service": "openai",
                    "retry_after": 60
                }
            }]
        }
    }
```

---

## In-Memory Session Models

### Conversation Session

```python
from dataclasses import dataclass
from datetime import datetime
from collections import deque
from typing import Dict, Any

@dataclass
class Message:
    """Single message in conversation"""
    role: str  # "user" or "assistant"
    content: str
    timestamp: datetime
    metadata: Dict[str, Any]  # agent_used, citations, etc.

class ConversationSession:
    """In-memory conversation session"""
    def __init__(
        self,
        session_id: str,
        user_id: str,
        expertise_level: ExpertiseLevel,
        max_turns: int = 10,
        timeout_minutes: int = 15
    ):
        self.session_id = session_id
        self.user_id = user_id
        self.expertise_level = expertise_level
        self.messages: deque[Message] = deque(maxlen=max_turns)
        self.created_at = datetime.now()
        self.last_activity = datetime.now()
        self.timeout_minutes = timeout_minutes
    
    def add_message(self, role: str, content: str, metadata: Dict[str, Any] = None):
        """Add message to conversation history"""
        msg = Message(
            role=role,
            content=content,
            timestamp=datetime.now(),
            metadata=metadata or {}
        )
        self.messages.append(msg)
        self.last_activity = datetime.now()
    
    def is_expired(self) -> bool:
        """Check if session has expired"""
        elapsed = (datetime.now() - self.last_activity).total_seconds()
        return elapsed > (self.timeout_minutes * 60)
    
    def get_context(self) -> List[Dict[str, str]]:
        """Get conversation history for agent context"""
        return [
            {"role": msg.role, "content": msg.content}
            for msg in self.messages
        ]
```

---

## Agent Tool Models

### Search Tool Input/Output

```python
class SearchToolInput(BaseModel):
    """Input for semantic search tool"""
    query: str
    expertise_level: ExpertiseLevel
    selected_text: Optional[str] = None
    max_chunks: Optional[int] = None  # Override default dynamic calculation

class RetrievedChunk(BaseModel):
    """Single retrieved chunk"""
    content: str
    score: float  # Similarity score
    chapter: str
    section: str
    line_range: str
    file_path: str

class SearchToolOutput(BaseModel):
    """Output from semantic search tool"""
    chunks: List[RetrievedChunk]
    total_retrieved: int
    search_strategy: str  # "semantic", "hybrid", "selected_text"
```

### Agent Context

```python
class AgentContext(BaseModel):
    """Shared context passed to all agents"""
    user_id: str
    session_id: str
    expertise_level: ExpertiseLevel
    conversation_history: List[Dict[str, str]]
    selected_text: Optional[str] = None
```

---

## Validation Rules

### Query Validation

- **Length**: 1-2000 characters
- **Rate limit**: Max 10 queries per minute per user (authenticated)
- **Content**: No SQL injection, XSS patterns
- **Language**: English only (for MVP)

### Selected Text Validation

- **Length**: Max 10,000 characters
- **Source**: Must be from book content (optional verification)

### Expertise Level

- **Enum validation**: Must be one of ["beginner", "intermediate", "advanced"]
- **Persistence**: Stored in Neon Postgres `users.expertiseLevel`

---

## State Transitions

### Conversation Session Lifecycle

```
[Created] 
   ↓ (first message)
[Active]
   ↓ (15 min inactivity)
[Expired]
   ↓ (cleanup task)
[Deleted]
```

### Message Flow

```
User Query
   ↓
[Validate] → (invalid) → [Error Response]
   ↓ (valid)
[Authenticate] → (unauthorized) → [401 Unauthorized]
   ↓ (authenticated)
[Triage Agent]
   ↓
[Specialist Agent] (Book Q&A or Selected Text)
   ↓
[Retrieve Chunks] (Qdrant search)
   ↓
[Generate Response] (OpenAI)
   ↓
[Format Citations]
   ↓
[Return Response]
```

---

## Data Retention

### In-Memory (Server Restart = Loss)

- **Conversation sessions**: 15 minutes after last activity
- **Message history**: Last 10 turns per session

### Persistent (Neon Postgres)

- **User expertise level**: Indefinite
- **Session tokens**: Managed by Better Auth (30-day sliding window)

### Vector Database (Qdrant)

- **Book content embeddings**: Persistent until manual re-indexing
- **No user data**: Qdrant only stores book content, not user queries

---

## Privacy & Security

### No Logging Policy

Per specification:
- ❌ No conversation content logging
- ❌ No query logging
- ❌ No user interaction tracking
- ✅ System metrics only (response time, error rates)

### Data Minimization

- Expertise level stored in database (necessary for feature)
- Session ID in memory only (not logged)
- Conversation history expires after 15 minutes
- No persistent conversation storage

### Security Measures

- All API requests require authentication
- Session validation on every request
- SQL injection prevention via parameterized queries
- XSS prevention via Pydantic validation
- Rate limiting per user

---

## Indexing Strategy

### Chunk Size Calculation

```python
CHUNK_CONFIG = {
    "max_tokens": 512,
    "overlap_tokens": 50,
    "min_chunk_size": 100,  # Minimum tokens per chunk
    "preserve_code_blocks": True,  # Don't split code
    "preserve_equations": True  # Don't split LaTeX
}
```

### Metadata Extraction

```python
def extract_metadata(file_path: str, chunk: str, start_line: int) -> dict:
    """Extract metadata from MDX file"""
    return {
        "file_path": file_path,
        "chapter": extract_chapter(file_path),
        "section": extract_section(chunk),
        "subsection": extract_subsection(chunk),
        "line_range": f"{start_line}-{start_line + chunk.count(chr(10))}",
        "token_count": count_tokens(chunk),
        "created_at": datetime.now().isoformat()
    }
```

---

## Next Steps

1. ✅ Data model defined
2. → Define API contracts (OpenAPI spec)
3. → Create quickstart guide
4. → Generate implementation tasks
