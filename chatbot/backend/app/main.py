"""Main FastAPI application entry point."""

from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from app.api import router
from app.core import settings
from app.db import postgres_service, qdrant_service
from app.services import rag_service


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Application lifespan manager for startup and shutdown events.

    Args:
        app: FastAPI application instance
    """
    # Startup
    print("🚀 Starting RAG Chatbot Backend...")

    # Initialize database tables
    print("📊 Ensuring database tables exist...")
    await postgres_service.ensure_tables_exist()

    # Initialize Qdrant collection
    print("🔍 Ensuring Qdrant collection exists...")
    await qdrant_service.ensure_collection_exists()

    # Initialize AI agents
    print("🤖 Initializing AI agents...")
    await rag_service.initialize_agents()

    print("✅ Application startup complete!")

    yield

    # Shutdown
    print("👋 Shutting down RAG Chatbot Backend...")
    await rag_service.cleanup_agents()
    print("✅ Cleanup complete!")


# Create FastAPI application
app = FastAPI(
    title="RAG Chatbot Backend",
    description="Backend API for the Interactive Book Learning RAG Chatbot",
    version="0.1.0",
    lifespan=lifespan,
)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "http://localhost:5173"],  # Frontend origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routes
app.include_router(router)


@app.get("/")
async def root() -> dict:
    """Root endpoint."""
    return {
        "message": "RAG Chatbot Backend API",
        "version": "0.1.0",
        "docs": "/docs",
    }
