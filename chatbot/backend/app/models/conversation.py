"""Conversation models and schemas."""

from datetime import datetime
from typing import List, Optional

from pydantic import BaseModel, Field


class Message(BaseModel):
    """Individual message in a conversation."""

    role: str = Field(..., description="Message role (user/assistant/system)")
    content: str = Field(..., description="Message content")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Message timestamp")


class Conversation(BaseModel):
    """Conversation session model."""

    session_id: str = Field(..., description="Unique session identifier")
    user_id: str = Field(..., description="User ID from Better Auth")
    messages: List[Message] = Field(default_factory=list, description="Conversation messages")
    created_at: datetime = Field(
        default_factory=datetime.utcnow, description="Session creation time"
    )
    last_activity: datetime = Field(
        default_factory=datetime.utcnow,
        description="Last activity timestamp",
    )
    is_active: bool = Field(default=True, description="Session active status")
    previous_response_id: Optional[str] = Field(
        default=None, description="OpenAI Agents SDK response ID for conversation continuity"
    )


class ConversationCreate(BaseModel):
    """Schema for creating a new conversation."""

    user_id: str = Field(..., description="User ID from Better Auth")


class ConversationResponse(BaseModel):
    """Response schema for conversation."""

    session_id: str = Field(..., description="Session identifier")
    messages: List[Message] = Field(..., description="Conversation messages")
    created_at: datetime = Field(..., description="Session creation time")
    last_activity: datetime = Field(..., description="Last activity timestamp")
