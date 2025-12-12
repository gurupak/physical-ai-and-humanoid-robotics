"""RAG service for orchestrating query processing and response generation."""

from typing import TYPE_CHECKING, List

from app.agents import book_qa_agent, selected_text_agent, triage_agent
from app.core import settings
from app.db import postgres_service
from app.models.conversation import Message
from app.models.query import ChatQuery, QueryContext, QueryResponse, QueryType
from app.utils import query_analyzer

if TYPE_CHECKING:
    from app.services.session import SessionService


class RAGService:
    """RAG service that orchestrates agents and manages query processing."""

    def __init__(self) -> None:
        """Initialize RAG service."""
        self.triage = triage_agent
        self.book_qa = book_qa_agent
        self.selected_text = selected_text_agent
        self._session_service = None
        self.postgres = postgres_service
        self.analyzer = query_analyzer

    @property
    def session_service(self) -> "SessionService":
        """Lazy-load session service to avoid circular import."""
        if self._session_service is None:
            from app.services.session import session_service

            self._session_service = session_service
        return self._session_service

    async def initialize_agents(self) -> None:
        """Initialize all AI agents."""
        await self.triage.initialize()
        await self.book_qa.initialize()
        await self.selected_text.initialize()

    async def process_query(
        self,
        query: ChatQuery,
        user_id: str,
    ) -> QueryResponse:
        """
        Process a chat query end-to-end.

        Args:
            query: Chat query from user
            user_id: User identifier

        Returns:
            Query response with assistant message
        """
        # Validate session
        session_valid = await self.session_service.validate_session(query.session_id)
        if not session_valid:
            raise ValueError("Invalid or expired session")

        # Get user preferences for expertise level
        user_prefs = await self.postgres.get_user_preferences(user_id)
        expertise_level = user_prefs["expertise_level"] if user_prefs else "beginner"

        # Determine query type
        query_type = self.analyzer.determine_query_type(
            query.message,
            query.selected_text,
        )

        # Analyze query complexity
        complexity = self.analyzer.analyze_complexity(query.message)

        # Calculate chunks to retrieve
        chunks_to_retrieve = self.analyzer.calculate_chunks_to_retrieve(
            complexity=complexity,
            expertise_level=expertise_level,
            min_chunks=settings.min_chunks,
            max_chunks=settings.max_chunks,
        )

        # Build query context
        context = QueryContext(
            query=query.message,
            query_type=query_type,
            complexity=complexity,
            chunks_to_retrieve=chunks_to_retrieve,
            selected_text=query.selected_text,
            expertise_level=expertise_level,
        )

        # Get conversation history
        conversation_history = await self.session_service.get_conversation_history(
            query.session_id,
        )

        # Route to appropriate agent and get response
        if query_type == QueryType.SELECTED_TEXT:
            response_text = await self.selected_text.answer_query(
                context,
                conversation_history,
            )
        else:
            response_text = await self.book_qa.answer_query(
                context,
                conversation_history,
            )

        # Save user message
        await self.session_service.add_message(
            session_id=query.session_id,
            role="user",
            content=query.message,
        )

        # Save assistant response
        await self.session_service.add_message(
            session_id=query.session_id,
            role="assistant",
            content=response_text,
        )

        return QueryResponse(
            session_id=query.session_id,
            message=response_text,
            chunks_retrieved=chunks_to_retrieve if query_type == QueryType.GENERAL else 0,
            query_type=query_type,
        )

    async def cleanup_agents(self) -> None:
        """Clean up all AI agents."""
        await self.triage.cleanup()
        await self.book_qa.cleanup()
        await self.selected_text.cleanup()


# Global RAG service instance
rag_service = RAGService()
