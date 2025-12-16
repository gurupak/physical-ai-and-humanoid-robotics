"""Session management service for conversation handling."""

import uuid
from datetime import datetime, timedelta
from typing import List, Optional

from app.core import settings
from app.db import postgres_service
from app.models.conversation import Conversation, Message


class SessionService:
    """Service for managing conversation sessions."""

    def __init__(self) -> None:
        """Initialize session service."""
        self.postgres = postgres_service
        self.timeout_minutes = settings.session_timeout_minutes
        self.max_turns = settings.max_conversation_turns

    async def create_session(self, user_id: str) -> str:
        """
        Create a new conversation session.

        Args:
            user_id: User identifier

        Returns:
            New session ID
        """
        session_id = str(uuid.uuid4())
        await self.postgres.create_session(session_id, user_id)
        return session_id

    async def get_session(self, session_id: str) -> Optional[Conversation]:
        """
        Get a conversation session with messages.

        Args:
            session_id: Session identifier

        Returns:
            Conversation object or None if not found/expired
        """
        session = await self.postgres.get_session(session_id)
        if not session:
            return None

        # Check if session is expired
        last_activity = session["last_activity"]
        timeout_threshold = datetime.utcnow() - timedelta(minutes=self.timeout_minutes)

        if last_activity < timeout_threshold or not session["is_active"]:
            return None

        # Get messages
        messages_data = await self.postgres.get_session_messages(session_id)
        messages = [
            Message(
                role=msg["role"],
                content=msg["content"],
                timestamp=msg["timestamp"],
            )
            for msg in messages_data
        ]

        return Conversation(
            session_id=session_id,
            user_id=session["user_id"],
            messages=messages,
            created_at=session["created_at"],
            last_activity=session["last_activity"],
            is_active=session["is_active"],
        )

    async def add_message(
        self,
        session_id: str,
        role: str,
        content: str,
    ) -> None:
        """
        Add a message to a session.

        Args:
            session_id: Session identifier
            role: Message role (user/assistant/system)
            content: Message content
        """
        await self.postgres.add_message(session_id, role, content)
        await self.postgres.update_session_activity(session_id)

    async def get_conversation_history(
        self,
        session_id: str,
        limit: Optional[int] = None,
    ) -> List[Message]:
        """
        Get conversation history for a session.

        Args:
            session_id: Session identifier
            limit: Optional limit on number of messages (defaults to max_turns)

        Returns:
            List of messages
        """
        if limit is None:
            limit = self.max_turns * 2  # user + assistant per turn

        messages_data = await self.postgres.get_session_messages(session_id, limit=limit)
        return [
            Message(
                role=msg["role"],
                content=msg["content"],
                timestamp=msg["timestamp"],
            )
            for msg in messages_data
        ]

    async def validate_session(self, session_id: str) -> bool:
        """
        Validate if a session exists and is active.

        Args:
            session_id: Session identifier

        Returns:
            True if session is valid, False otherwise
        """
        session = await self.get_session(session_id)
        return session is not None

    async def cleanup_expired_sessions(self) -> None:
        """Deactivate expired sessions based on timeout."""
        await self.postgres.deactivate_expired_sessions(self.timeout_minutes)


# Global session service instance
session_service = SessionService()
