"""RAG (Retrieval-Augmented Generation) tools for OpenAI Agents SDK."""

import logging
from typing import Optional

from agents import function_tool
from agents.tool_context import ToolContext

logger = logging.getLogger(__name__)


@function_tool
async def search_book_content(
    query: str, num_chunks: int = 5, expertise_level: str = "intermediate"
) -> str:
    """
    Search the Physical AI and Humanoid Robotics book content using RAG.

    This tool performs semantic search over the book content to find relevant
    information that can help answer the user's question.

    Args:
        query: The search query to find relevant book content
        num_chunks: Number of relevant chunks to retrieve (3-10, default 5)
        expertise_level: User's expertise level (beginner/intermediate/advanced)

    Returns:
        Formatted context from retrieved book chunks with relevance scores
    """
    from app.db.qdrant_client import qdrant_service
    from app.services.embedding import embedding_service

    logger.info(f"Searching book content for query: {query[:50]}...")

    try:
        # Validate num_chunks
        num_chunks = max(3, min(10, num_chunks))

        # Generate embedding for the query
        query_embedding = await embedding_service.generate_embedding(query)

        # Search Qdrant for similar chunks
        chunks = await qdrant_service.search_similar_chunks(
            query_embedding=query_embedding, limit=num_chunks, score_threshold=0.7
        )

        # Format chunks as context
        if not chunks:
            return "No relevant content found in the book for this query."

        context_parts = []
        context_parts.append(f"Found {len(chunks)} relevant sections from the book:\n")

        for i, chunk in enumerate(chunks, 1):
            section_title = chunk.payload.get("section_title", "Unknown Section")
            content = chunk.payload.get("content", "")
            score = chunk.score

            context_parts.append(
                f"\n[Section {i}] {section_title} (Relevance: {score:.2f})\n{content}\n"
            )

        context_parts.append(
            f"\nUse the above sections to answer the user's question. "
            f"Cite the section numbers in your response."
        )

        result = "\n".join(context_parts)
        logger.info(f"Successfully retrieved {len(chunks)} chunks")
        return result

    except Exception as e:
        logger.error(f"Error searching book content: {str(e)}", exc_info=True)
        return f"Error searching book content: {str(e)}"


@function_tool
async def get_user_expertise_level(ctx: ToolContext) -> str:
    """
    Get the logged-in user's expertise level to tailor response complexity.

    The expertise level helps adapt explanations to the user's knowledge level:
    - beginner: Simple explanations with more context
    - intermediate: Balanced technical depth
    - advanced: Technical details and advanced concepts

    This function automatically retrieves the user_id from the logged-in session context.
    No parameters are needed.

    Returns:
        User's expertise level: "beginner", "intermediate", or "advanced"
    """
    from app.db.postgres import postgres_service

    # Get user_id from context - context is a Pydantic BaseModel with user_id attribute
    logger.info(f"Tool context type: {type(ctx.context)}")
    logger.info(f"Tool context value: {ctx.context}")

    if not ctx.context:
        logger.warning("No context provided, defaulting to intermediate")
        return "intermediate"

    # Access user_id from the Pydantic model
    user_id = ctx.context.user_id
    logger.info(f"Getting expertise level for user: {user_id}")

    try:
        preferences = await postgres_service.get_user_preferences(user_id)

        if preferences and "expertise_level" in preferences:
            level = preferences["expertise_level"]
            logger.info(f"User {user_id} expertise level: {level}")
            return level
        else:
            logger.info(f"No preferences found for user {user_id}, defaulting to intermediate")
            return "intermediate"

    except Exception as e:
        logger.error(f"Error getting user expertise level: {str(e)}", exc_info=True)
        return "intermediate"  # Default fallback


@function_tool
async def format_selected_text_context(
    selected_text: str, question: str, expertise_level: str = "intermediate"
) -> str:
    """
    Format selected text with the user's question for explanation.

    This tool structures the selected text and question for the agent to provide
    a clear, focused explanation.

    Args:
        selected_text: The text the user selected from the book
        question: The user's question about the selected text
        expertise_level: User's expertise level for response tailoring

    Returns:
        Formatted context for the agent to work with
    """
    logger.info("Formatting selected text context")

    return f"""
The user has selected this text from the book:

--- SELECTED TEXT ---
{selected_text}
--- END SELECTED TEXT ---

User's Question: {question}

User's Expertise Level: {expertise_level}

Please provide a clear explanation that:
1. Directly addresses the selected text
2. Answers the user's specific question
3. Adapts complexity to the user's expertise level ({expertise_level})
4. Uses examples or analogies if helpful
"""
