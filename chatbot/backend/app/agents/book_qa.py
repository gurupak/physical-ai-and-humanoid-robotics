"""Book Q&A agent for answering general questions about book content."""

from typing import List

from openai import AsyncOpenAI
from openai.types.beta import Assistant

from app.core import settings
from app.db import qdrant_service
from app.models.conversation import Message
from app.models.query import QueryContext, RetrievedChunk
from app.services import embedding_service


class BookQAAgent:
    """Specialist agent for answering general book questions using RAG."""

    def __init__(self) -> None:
        """Initialize the Book Q&A agent."""
        self.client = AsyncOpenAI(api_key=settings.openai_api_key)
        self.assistant: Assistant | None = None
        self.qdrant = qdrant_service
        self.embedding_service = embedding_service

    async def initialize(self) -> None:
        """Create the OpenAI assistant for book Q&A."""
        instructions = """You are an expert AI learning assistant specializing in explaining AI concepts from an educational book.

Your responsibilities:
- Answer questions about AI concepts, techniques, and implementations covered in the book
- Provide clear, accurate explanations tailored to the user's expertise level
- Use the retrieved book content as your primary knowledge source
- If information isn't in the provided context, acknowledge this clearly
- Structure explanations progressively, starting with fundamentals
- Use examples and analogies when helpful

Guidelines:
- Always base answers on the provided book content
- Adapt explanation depth to user's expertise level
- Be concise but thorough
- When concepts are complex, break them into digestible parts
- Encourage follow-up questions for clarification"""

        self.assistant = await self.client.beta.assistants.create(
            name="Book Q&A Agent",
            instructions=instructions,
            model="gpt-4o",
        )

    async def answer_query(
        self,
        context: QueryContext,
        conversation_history: List[Message],
    ) -> str:
        """
        Answer a general book question using RAG.

        Args:
            context: Query context with parameters
            conversation_history: Recent conversation messages

        Returns:
            Assistant's response
        """
        # Generate embedding for the query
        query_embedding = await self.embedding_service.generate_embedding(context.query)

        # Retrieve relevant chunks from Qdrant
        retrieved_chunks = await self.qdrant.search_similar_chunks(
            query_embedding=query_embedding,
            limit=context.chunks_to_retrieve,
            score_threshold=0.7,
        )

        # Build the context from retrieved chunks
        context_text = self._build_context(retrieved_chunks, context.expertise_level)

        # Build the prompt with conversation history
        messages = self._build_messages(
            context.query,
            context_text,
            conversation_history,
            context.expertise_level,
        )

        # Get response from OpenAI
        response = await self.client.chat.completions.create(
            model="gpt-4o",
            messages=messages,
            temperature=0.7,
            max_tokens=1000,
        )

        return response.choices[0].message.content or ""

    def _build_context(
        self,
        chunks: List[RetrievedChunk],
        expertise_level: str,
    ) -> str:
        """
        Build context string from retrieved chunks.

        Args:
            chunks: Retrieved document chunks
            expertise_level: User's expertise level

        Returns:
            Formatted context string
        """
        if not chunks:
            return "No relevant book content found."

        context_parts = [
            f"Book Content (Expertise Level: {expertise_level}):",
            "",
        ]

        for i, chunk in enumerate(chunks, 1):
            section_info = ""
            if "title" in chunk.metadata:
                section_info = f"Section: {chunk.metadata['title']}"

            context_parts.append(f"[{i}] {section_info}")
            context_parts.append(chunk.content)
            context_parts.append("")

        return "\n".join(context_parts)

    def _build_messages(
        self,
        query: str,
        context: str,
        history: List[Message],
        expertise_level: str,
    ) -> List[dict]:
        """
        Build message list for chat completion.

        Args:
            query: User query
            context: Retrieved context
            history: Conversation history
            expertise_level: User's expertise level

        Returns:
            List of message dicts for OpenAI API
        """
        messages = [
            {
                "role": "system",
                "content": f"""You are an AI learning assistant EXCLUSIVELY for an educational book about AI, Robotics, ROS2, Isaac Sim, VSLAM, Navigation, and Vision-Language-Action models.

CRITICAL RULES:
1. ONLY answer questions about topics covered in the book content provided below
2. If the question is NOT about the book content, politely redirect: "I'm specifically designed to help with this book's content on AI, Robotics, ROS2, and related topics. Please ask me about these subjects!"
3. NEVER answer general questions unrelated to the book (math, history, etc.)
4. Base ALL answers strictly on the retrieved book content below
5. If the book content doesn't contain the answer, say: "I don't see that topic covered in this book. Could you ask about robotics, ROS2, Isaac Sim, VSLAM, or VLA models?"

User expertise level: {expertise_level}

BOOK CONTENT:
{context}

Provide a clear answer based ONLY on the book content above. Tailor the explanation depth to the user's expertise level.""",
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


# Global Book Q&A agent instance
book_qa_agent = BookQAAgent()
