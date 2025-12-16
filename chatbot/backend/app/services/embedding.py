"""OpenAI embedding service for generating text embeddings."""

from typing import List

from openai import AsyncOpenAI

from app.core import settings


class EmbeddingService:
    """Service for generating embeddings using OpenAI's embedding models."""

    def __init__(self) -> None:
        """Initialize OpenAI client."""
        self.client = AsyncOpenAI(api_key=settings.openai_api_key)
        self.model = settings.embedding_model

    async def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a single text.

        Args:
            text: Input text

        Returns:
            Embedding vector
        """
        response = await self.client.embeddings.create(
            model=self.model,
            input=text,
        )
        return response.data[0].embedding

    async def generate_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts in batch.

        Args:
            texts: List of input texts

        Returns:
            List of embedding vectors
        """
        if not texts:
            return []

        response = await self.client.embeddings.create(
            model=self.model,
            input=texts,
        )

        # Sort by index to ensure correct order
        sorted_data = sorted(response.data, key=lambda x: x.index)
        return [item.embedding for item in sorted_data]


# Global embedding service instance
embedding_service = EmbeddingService()
