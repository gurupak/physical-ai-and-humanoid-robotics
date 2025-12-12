"""Selected Text agent for answering questions about user-selected text."""

from typing import List

from openai import AsyncOpenAI
from openai.types.beta import Assistant

from app.core import settings
from app.models.conversation import Message
from app.models.query import QueryContext


class SelectedTextAgent:
    """Specialist agent for answering questions about selected text from the book."""

    def __init__(self) -> None:
        """Initialize the Selected Text agent."""
        self.client = AsyncOpenAI(api_key=settings.openai_api_key)
        self.assistant: Assistant | None = None

    async def initialize(self) -> None:
        """Create the OpenAI assistant for selected text queries."""
        instructions = """You are an expert AI learning assistant specializing in explaining specific text selections from an educational book.

Your responsibilities:
- Answer questions about the selected text provided by the user
- Clarify concepts, terminology, and implementation details in the selection
- Provide context and explain how the selected part fits into broader concepts
- Tailor explanations to the user's expertise level

Guidelines:
- Focus primarily on the selected text
- Provide additional context when it helps understanding
- Break down complex passages into understandable components
- Use examples to illustrate points when helpful
- Adapt explanation depth to user's expertise level"""

        self.assistant = await self.client.beta.assistants.create(
            name="Selected Text Agent",
            instructions=instructions,
            model="gpt-4o",
        )

    async def answer_query(
        self,
        context: QueryContext,
        conversation_history: List[Message],
    ) -> str:
        """
        Answer a question about selected text.

        Args:
            context: Query context with selected text
            conversation_history: Recent conversation messages

        Returns:
            Assistant's response
        """
        if not context.selected_text:
            return (
                "No text was selected. Please select text from the book to ask questions about it."
            )

        # Build messages for chat completion
        messages = self._build_messages(
            query=context.query,
            selected_text=context.selected_text,
            history=conversation_history,
            expertise_level=context.expertise_level,
        )

        # Get response from OpenAI
        response = await self.client.chat.completions.create(
            model="gpt-4o",
            messages=messages,
            temperature=0.7,
            max_tokens=1000,
        )

        return response.choices[0].message.content or ""

    def _build_messages(
        self,
        query: str,
        selected_text: str,
        history: List[Message],
        expertise_level: str,
    ) -> List[dict]:
        """
        Build message list for chat completion.

        Args:
            query: User query
            selected_text: Text selected from the book
            history: Conversation history
            expertise_level: User's expertise level

        Returns:
            List of message dicts for OpenAI API
        """
        messages = [
            {
                "role": "system",
                "content": f"""You are an expert AI learning assistant.
User expertise level: {expertise_level}

SELECTED TEXT FROM BOOK:
---
{selected_text}
---

Answer the user's question about this selected text. Provide clear explanations
tailored to their expertise level ({expertise_level}).""",
            },
        ]

        # Add recent conversation history (limited)
        for msg in history[-10:]:  # Last 5 turns (user + assistant)
            messages.append(
                {
                    "role": msg.role,
                    "content": msg.content,
                }
            )

        # Add current query
        messages.append(
            {
                "role": "user",
                "content": query,
            }
        )

        return messages

    async def cleanup(self) -> None:
        """Clean up the assistant."""
        if self.assistant:
            await self.client.beta.assistants.delete(self.assistant.id)


# Global Selected Text agent instance
selected_text_agent = SelectedTextAgent()
