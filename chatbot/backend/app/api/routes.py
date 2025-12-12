"""API routes for the chatbot backend."""

from typing import Optional

from fastapi import APIRouter, Header, HTTPException

from app.db import postgres_service
from app.models.conversation import ConversationCreate, ConversationResponse
from app.models.query import ChatQuery, QueryResponse
from app.models.user import UserPreferencesCreate, UserPreferencesUpdate
from app.services import rag_service, session_service

router = APIRouter(prefix="/api/v1")


@router.post("/sessions", response_model=ConversationResponse)
async def create_session(
    session_data: ConversationCreate,
) -> ConversationResponse:
    """
    Create a new conversation session.

    Args:
        session_data: Session creation data with user_id

    Returns:
        New session details
    """
    try:
        session_id = await session_service.create_session(session_data.user_id)
        session = await session_service.get_session(session_id)

        if not session:
            raise HTTPException(status_code=500, detail="Failed to create session")

        return ConversationResponse(
            session_id=session.session_id,
            messages=session.messages,
            created_at=session.created_at,
            last_activity=session.last_activity,
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/sessions/{session_id}", response_model=ConversationResponse)
async def get_session(session_id: str) -> ConversationResponse:
    """
    Get a conversation session by ID.

    Args:
        session_id: Session identifier

    Returns:
        Session details with messages
    """
    session = await session_service.get_session(session_id)

    if not session:
        raise HTTPException(status_code=404, detail="Session not found or expired")

    return ConversationResponse(
        session_id=session.session_id,
        messages=session.messages,
        created_at=session.created_at,
        last_activity=session.last_activity,
    )


@router.post("/chat", response_model=QueryResponse)
async def chat(
    query: ChatQuery,
    user_id: Optional[str] = Header(None, alias="X-User-ID"),
) -> QueryResponse:
    """
    Process a chat query and get a response.

    Args:
        query: Chat query with session_id and message
        user_id: User ID from authentication header

    Returns:
        Query response with assistant message
    """
    if not user_id:
        raise HTTPException(status_code=401, detail="User ID required in X-User-ID header")

    try:
        response = await rag_service.process_query(query, user_id)
        return response
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/users/preferences")
async def create_user_preferences(
    preferences: UserPreferencesCreate,
    user_id: Optional[str] = Header(None, alias="X-User-ID"),
) -> dict:
    """
    Create or update user preferences.

    Args:
        preferences: User preferences data
        user_id: User ID from authentication header

    Returns:
        Success message
    """
    if not user_id:
        raise HTTPException(status_code=401, detail="User ID required in X-User-ID header")

    try:
        await postgres_service.upsert_user_preferences(
            user_id=user_id,
            expertise_level=preferences.expertise_level,
        )
        return {"message": "Preferences updated successfully"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/users/preferences")
async def get_user_preferences(
    user_id: Optional[str] = Header(None, alias="X-User-ID"),
) -> dict:
    """
    Get user preferences.

    Args:
        user_id: User ID from authentication header

    Returns:
        User preferences
    """
    if not user_id:
        raise HTTPException(status_code=401, detail="User ID required in X-User-ID header")

    try:
        prefs = await postgres_service.get_user_preferences(user_id)
        if not prefs:
            return {"expertise_level": "beginner"}
        return dict(prefs)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.patch("/users/preferences")
async def update_user_preferences(
    preferences: UserPreferencesUpdate,
    user_id: Optional[str] = Header(None, alias="X-User-ID"),
) -> dict:
    """
    Update user preferences.

    Args:
        preferences: Updated preferences
        user_id: User ID from authentication header

    Returns:
        Success message
    """
    if not user_id:
        raise HTTPException(status_code=401, detail="User ID required in X-User-ID header")

    if not preferences.expertise_level:
        raise HTTPException(status_code=400, detail="No fields to update")

    try:
        await postgres_service.upsert_user_preferences(
            user_id=user_id,
            expertise_level=preferences.expertise_level,
        )
        return {"message": "Preferences updated successfully"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/health")
async def health_check() -> dict:
    """Health check endpoint."""
    return {"status": "healthy"}
