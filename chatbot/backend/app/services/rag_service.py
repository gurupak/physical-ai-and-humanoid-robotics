"""RAG service using OpenAI Agents SDK for query processing."""

import logging
from typing import TYPE_CHECKING

from agents import Runner
from pydantic import BaseModel

from app.agents.agent_definitions import triage_agent
from app.db import postgres_service
from app.models.query import ChatQuery, QueryResponse

if TYPE_CHECKING:
    from app.services.session import SessionService

logger = logging.getLogger(__name__)


class UserContext(BaseModel):
    """Context passed to agents containing user information."""

    user_id: str


class RAGService:
    """
    RAG Service using OpenAI Agents SDK.

    Simplified orchestration - agents handle all the complexity through:
    - Automatic tool calling (RAG search, user preferences)
    - Dynamic handoffs between specialist agents
    - Native conversation continuity via response IDs
    """

    def __init__(self) -> None:
        """Initialize RAG service."""
        self._session_service = None
        self.postgres = postgres_service

    @property
    def session_service(self) -> "SessionService":
        """Lazy-load session service to avoid circular import."""
        if self._session_service is None:
            from app.services.session import session_service

            self._session_service = session_service
        return self._session_service

    async def initialize_agents(self) -> None:
        """
        Initialize agents (no-op for OpenAI Agents SDK).
        Agents are defined at import time.
        """
        logger.info("RAG Service initialized with OpenAI Agents SDK")
        logger.info("Agents: Triage → [Book Q&A, Selected Text Explainer]")

    async def cleanup_agents(self) -> None:
        """Cleanup agents (no-op for OpenAI Agents SDK)."""
        logger.info("RAG Service cleanup complete")

    async def process_query(
        self,
        query: ChatQuery,
        user_id: str,
    ) -> QueryResponse:
        """
        Process a user query using OpenAI Agents SDK.

        The triage agent will automatically:
        1. Determine which specialist agent to use
        2. Hand off to that agent
        3. The specialist agent will call tools as needed (RAG search, etc.)
        4. Return the final response

        Args:
            query: The chat query with message, session_id, optional selected_text
            user_id: The user's identifier

        Returns:
            QueryResponse with the agent's answer

        Raises:
            ValueError: If session is invalid
            Exception: For other errors
        """
        try:
            # 1. Validate session exists and belongs to user
            session = await self.session_service.get_session(query.session_id)

            if not session:
                raise ValueError(f"Session {query.session_id} not found")

            if session.user_id != user_id:
                raise ValueError("Session does not belong to user")

            # 2. Build input for agent
            user_message = query.message

            # If there's selected text, include it in the message context
            if query.selected_text:
                user_message = f"""
I have selected this text from the book:
"{query.selected_text}"

My question: {query.message}
"""
                logger.info(
                    f"Processing query with selected text (length: {len(query.selected_text)})"
                )
            else:
                logger.info(f"Processing general query: {query.message[:100]}")

            # 3. Get previous response ID for conversation continuity
            previous_response_id = session.previous_response_id

            if previous_response_id:
                logger.info(
                    f"Continuing conversation with response_id: {previous_response_id[:20]}..."
                )
            else:
                logger.info("Starting new conversation")

            # 4. Run the agent using OpenAI Agents SDK with user context
            logger.info(f"Running triage agent for session {query.session_id}")

            # Create context with user_id for tool access
            user_context = UserContext(user_id=user_id)

            result = await Runner.run(
                triage_agent,
                user_message,
                context=user_context,
                previous_response_id=previous_response_id,
            )

            # 5. Extract response
            assistant_message = result.final_output
            new_response_id = result.last_response_id

            # Debug logging
            logger.info(f"Result object: {result}")
            logger.info(f"Final output: {assistant_message}")
            logger.info(f"Last response ID: {new_response_id}")
            logger.info(
                f"Agent response generated (response_id: {new_response_id[:20] if new_response_id else 'None'}...)"
            )

            # 6. Update session with new response ID for next turn
            await self.postgres.update_session_response_id(
                session_id=query.session_id, response_id=new_response_id
            )

            # 7. Save messages to DB for analytics/debugging
            # (Agent SDK manages conversation internally, but we log for monitoring)
            await self._save_messages_for_analytics(
                session_id=query.session_id,
                user_message=query.message,  # Original message without formatting
                assistant_message=assistant_message,
            )

            # 8. Return response
            return QueryResponse(
                session_id=query.session_id,
                message=assistant_message,
                query_type="agent_handled",  # Agent SDK handled routing
                chunks_retrieved=None,  # Unknown - agent handles internally
                response_id=new_response_id,
            )

        except ValueError as e:
            # Re-raise validation errors
            logger.error(f"Validation error: {str(e)}")
            raise

        except Exception as e:
            logger.error(f"Error processing query: {str(e)}", exc_info=True)
            raise Exception(f"Failed to process query: {str(e)}")

    async def _save_messages_for_analytics(
        self, session_id: str, user_message: str, assistant_message: str
    ) -> None:
        """
        Save messages to database for analytics and debugging.

        Note: OpenAI Agents SDK manages conversation history internally via response_id.
        This is just for logging/analytics purposes.

        Args:
            session_id: Session identifier
            user_message: User's message
            assistant_message: Assistant's response
        """
        try:
            # Save user message
            await self.postgres.add_message(
                session_id=session_id, role="user", content=user_message
            )

            # Save assistant message
            await self.postgres.add_message(
                session_id=session_id, role="assistant", content=assistant_message
            )

            logger.debug(f"Messages saved for analytics (session: {session_id})")

        except Exception as e:
            # Non-critical - log warning but don't fail the request
            logger.warning(f"Failed to save messages for analytics: {str(e)}")


# Global service instance
rag_service = RAGService()
