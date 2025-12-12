"""Qdrant vector database client wrapper."""

from typing import List, Optional

from qdrant_client import QdrantClient
from qdrant_client.models import Distance, PointStruct, VectorParams

from app.core import settings
from app.models.query import RetrievedChunk


class QdrantService:
    """Qdrant vector database service for managing book content embeddings."""

    def __init__(self) -> None:
        """Initialize Qdrant client."""
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            timeout=120,  # Increase timeout to 120 seconds for cloud uploads
        )
        self.collection_name = settings.qdrant_collection_name

    async def ensure_collection_exists(self, vector_size: int = 1536) -> None:
        """
        Ensure the collection exists, create if it doesn't.

        Args:
            vector_size: Size of embedding vectors (default: 1536 for text-embedding-3-small)
        """
        collections = self.client.get_collections().collections
        collection_names = [col.name for col in collections]

        if self.collection_name not in collection_names:
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(size=vector_size, distance=Distance.COSINE),
            )

    async def upsert_chunks(
        self,
        chunks: List[str],
        embeddings: List[List[float]],
        metadata: List[dict],
        start_id: int = 0,
    ) -> None:
        """
        Insert or update document chunks with their embeddings.

        Args:
            chunks: List of text chunks
            embeddings: List of embedding vectors
            metadata: List of metadata dictionaries for each chunk
            start_id: Starting ID for the batch (for proper ID assignment across batches)
        """
        points = [
            PointStruct(
                id=start_id + i,
                vector=embedding,
                payload={"content": chunk, **meta},
            )
            for i, (chunk, embedding, meta) in enumerate(zip(chunks, embeddings, metadata))
        ]

        self.client.upsert(
            collection_name=self.collection_name,
            points=points,
        )

    async def search_similar_chunks(
        self,
        query_embedding: List[float],
        limit: int = 5,
        score_threshold: float = 0.7,
    ) -> List[RetrievedChunk]:
        """
        Search for similar chunks using vector similarity.

        Args:
            query_embedding: Query embedding vector
            limit: Maximum number of results to return
            score_threshold: Minimum similarity score threshold

        Returns:
            List of retrieved chunks with scores and metadata
        """
        try:
            search_results = self.client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                limit=limit,
                score_threshold=score_threshold,
            ).points
        except AttributeError:
            # Fallback for older API
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=limit,
                score_threshold=score_threshold,
            )

        return [
            RetrievedChunk(
                content=result.payload.get("content", ""),
                score=result.score,
                metadata={k: v for k, v in result.payload.items() if k != "content"},
            )
            for result in search_results
        ]

    async def delete_collection(self) -> None:
        """Delete the collection (use with caution)."""
        self.client.delete_collection(collection_name=self.collection_name)

    async def get_collection_info(self) -> Optional[dict]:
        """Get information about the collection."""
        try:
            info = self.client.get_collection(collection_name=self.collection_name)
            return {
                "points_count": info.points_count,
                "status": info.status,
            }
        except Exception:
            return None


# Global Qdrant service instance
qdrant_service = QdrantService()
