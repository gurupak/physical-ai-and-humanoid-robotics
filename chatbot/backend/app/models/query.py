"""Query models and schemas."""

from enum import Enum
from typing import List, Optional

from pydantic import BaseModel, Field


class QueryType(str, Enum):
    """Query type classification."""

    GENERAL = "general"
    SELECTED_TEXT = "selected_text"


class QueryComplexity(str, Enum):
    """Query complexity classification."""

    SIMPLE = "simple"
    MODERATE = "moderate"
    COMPLEX = "complex"


class ChatQuery(BaseModel):
    """Chat query request schema."""

    session_id: str = Field(..., description="Session identifier")
    message: str = Field(..., min_length=1, description="User query message")
    selected_text: Optional[str] = Field(None, description="Selected text from book (if any)")


class RetrievedChunk(BaseModel):
    """Retrieved document chunk schema."""

    content: str = Field(..., description="Chunk content")
    score: float = Field(..., description="Similarity score")
    metadata: dict = Field(default_factory=dict, description="Chunk metadata")


class QueryResponse(BaseModel):
    """Query response schema."""

    session_id: str = Field(..., description="Session identifier")
    message: str = Field(..., description="Assistant response message")
    chunks_retrieved: int = Field(..., description="Number of chunks retrieved")
    query_type: QueryType = Field(..., description="Type of query processed")

    class Config:
        """Pydantic config."""

        use_enum_values = True


class QueryContext(BaseModel):
    """Internal query context for processing."""

    query: str = Field(..., description="User query")
    query_type: QueryType = Field(..., description="Query type")
    complexity: QueryComplexity = Field(..., description="Query complexity")
    chunks_to_retrieve: int = Field(..., description="Number of chunks to retrieve")
    selected_text: Optional[str] = Field(None, description="Selected text context")
    expertise_level: str = Field(..., description="User expertise level")

    class Config:
        """Pydantic config."""

        use_enum_values = True
