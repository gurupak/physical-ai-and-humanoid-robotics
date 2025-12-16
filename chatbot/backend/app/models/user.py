"""User models and schemas."""

from enum import Enum
from typing import Optional

from pydantic import BaseModel, Field


class ExpertiseLevel(str, Enum):
    """User expertise level enum."""

    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"


class UserPreferences(BaseModel):
    """User preferences schema."""

    user_id: str = Field(..., description="User ID from Better Auth")
    expertise_level: ExpertiseLevel = Field(
        default=ExpertiseLevel.BEGINNER,
        description="User's expertise level",
    )

    class Config:
        """Pydantic config."""

        use_enum_values = True


class UserPreferencesCreate(BaseModel):
    """Schema for creating user preferences."""

    expertise_level: ExpertiseLevel = Field(
        default=ExpertiseLevel.BEGINNER,
        description="User's expertise level",
    )

    class Config:
        """Pydantic config."""

        use_enum_values = True


class UserPreferencesUpdate(BaseModel):
    """Schema for updating user preferences."""

    expertise_level: Optional[ExpertiseLevel] = Field(
        None,
        description="User's expertise level",
    )

    class Config:
        """Pydantic config."""

        use_enum_values = True
