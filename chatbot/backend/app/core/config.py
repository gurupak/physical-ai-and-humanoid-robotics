"""Application configuration management using Pydantic Settings."""

from pydantic import Field
from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        case_sensitive=False,
        extra="ignore",
    )

    # OpenAI Configuration
    openai_api_key: str = Field(..., description="OpenAI API key")

    # Qdrant Configuration
    qdrant_url: str = Field(..., description="Qdrant Cloud cluster URL")
    qdrant_api_key: str = Field(..., description="Qdrant API key")
    qdrant_collection_name: str = Field(
        default="book_content",
        description="Qdrant collection name for book content",
    )

    # Database Configuration
    database_url: str = Field(..., description="Neon Postgres connection string")

    # Application Configuration
    node_env: str = Field(default="development", description="Environment (development/production)")
    port: int = Field(default=8000, description="API server port")

    # RAG Configuration
    embedding_model: str = Field(
        default="text-embedding-3-small",
        description="OpenAI embedding model",
    )
    chunk_size: int = Field(default=1000, description="Default chunk size for text splitting")
    chunk_overlap: int = Field(default=200, description="Overlap between chunks")
    min_chunks: int = Field(default=3, description="Minimum chunks to retrieve")
    max_chunks: int = Field(default=10, description="Maximum chunks to retrieve")

    # Session Configuration
    session_timeout_minutes: int = Field(
        default=15,
        description="Session timeout in minutes",
    )
    max_conversation_turns: int = Field(
        default=10,
        description="Maximum conversation history turns",
    )

    @property
    def is_production(self) -> bool:
        """Check if running in production mode."""
        return self.node_env.lower() == "production"


# Global settings instance
settings = Settings()
