"""Triage agent for routing user queries to specialist agents."""

from openai import AsyncOpenAI
from openai.types.beta import Assistant

from app.core import settings
from app.models.query import QueryType


class TriageAgent:
    """Triage agent that routes queries to appropriate specialist agents."""

    SYSTEM_PROMPT = """You are a triage assistant for an AI learning book chatbot.

Your role is to analyze user queries and route them to the appropriate specialist:
- Book Q&A Agent: For general questions about book content, concepts, or explanations
- Selected Text Agent: For questions specifically about highlighted/selected text from the book

Analyze the user's query and determine:
1. Is this a general book question or about specific selected text?
2. What is the user trying to learn or understand?

Always route to the most appropriate specialist based on the query context."""

    def __init__(self) -> None:
        """Initialize the triage agent."""
        self.client = AsyncOpenAI(api_key=settings.openai_api_key)
        self.assistant: Assistant | None = None

    async def initialize(self) -> None:
        """Create the OpenAI assistant for triage."""
        self.assistant = await self.client.beta.assistants.create(
            name="Triage Agent",
            instructions=self.SYSTEM_PROMPT,
            model="gpt-4o-mini",
        )

    async def route_query(
        self,
        query: str,
        selected_text: str | None,
    ) -> QueryType:
        """
        Route a query to the appropriate specialist agent.

        Args:
            query: User query
            selected_text: Selected text from book (if any)

        Returns:
            QueryType indicating which specialist to use
        """
        # Simple routing logic
        if selected_text:
            return QueryType.SELECTED_TEXT
        return QueryType.GENERAL

    async def cleanup(self) -> None:
        """Clean up the assistant."""
        if self.assistant:
            await self.client.beta.assistants.delete(self.assistant.id)


# Global triage agent instance
triage_agent = TriageAgent()
