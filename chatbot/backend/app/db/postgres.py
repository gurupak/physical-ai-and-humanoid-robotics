"""PostgreSQL database connection and utilities."""

import psycopg2
from psycopg2.extras import RealDictCursor

from app.core import settings


class PostgresService:
    """PostgreSQL database service for user preferences and conversation history."""

    def __init__(self) -> None:
        """Initialize database connection."""
        self.connection_string = settings.database_url

    def get_connection(self):
        """Get a database connection."""
        return psycopg2.connect(self.connection_string, cursor_factory=RealDictCursor)

    async def ensure_tables_exist(self) -> None:
        """Ensure required tables exist, create if they don't."""
        with self.get_connection() as conn:
            with conn.cursor() as cur:
                # User preferences table
                cur.execute("""
                    CREATE TABLE IF NOT EXISTS user_preferences (
                        user_id VARCHAR(255) PRIMARY KEY,
                        expertise_level VARCHAR(50) NOT NULL DEFAULT 'beginner',
                        created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                        updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                    )
                """)

                # Conversation sessions table
                cur.execute("""
                    CREATE TABLE IF NOT EXISTS conversation_sessions (
                        session_id VARCHAR(255) PRIMARY KEY,
                        user_id VARCHAR(255) NOT NULL,
                        created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                        last_activity TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                        is_active BOOLEAN DEFAULT TRUE
                    )
                """)

                # Conversation messages table
                cur.execute("""
                    CREATE TABLE IF NOT EXISTS conversation_messages (
                        id SERIAL PRIMARY KEY,
                        session_id VARCHAR(255) NOT NULL,
                        role VARCHAR(50) NOT NULL,
                        content TEXT NOT NULL,
                        timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                        FOREIGN KEY (session_id) REFERENCES conversation_sessions(session_id)
                            ON DELETE CASCADE
                    )
                """)

                # Create indexes for better query performance
                cur.execute("""
                    CREATE INDEX IF NOT EXISTS idx_sessions_user_id
                    ON conversation_sessions(user_id)
                """)

                cur.execute("""
                    CREATE INDEX IF NOT EXISTS idx_messages_session_id
                    ON conversation_messages(session_id)
                """)

                cur.execute("""
                    CREATE INDEX IF NOT EXISTS idx_sessions_last_activity
                    ON conversation_sessions(last_activity)
                """)

                conn.commit()

    async def get_user_preferences(self, user_id: str) -> dict | None:
        """
        Get user preferences by user ID.

        Args:
            user_id: User identifier

        Returns:
            User preferences dict or None if not found
        """
        with self.get_connection() as conn:
            with conn.cursor() as cur:
                cur.execute(
                    "SELECT * FROM user_preferences WHERE user_id = %s",
                    (user_id,),
                )
                return cur.fetchone()

    async def upsert_user_preferences(
        self,
        user_id: str,
        expertise_level: str,
    ) -> None:
        """
        Insert or update user preferences.

        Args:
            user_id: User identifier
            expertise_level: User's expertise level
        """
        with self.get_connection() as conn:
            with conn.cursor() as cur:
                cur.execute(
                    """
                    INSERT INTO user_preferences (user_id, expertise_level, updated_at)
                    VALUES (%s, %s, CURRENT_TIMESTAMP)
                    ON CONFLICT (user_id)
                    DO UPDATE SET
                        expertise_level = EXCLUDED.expertise_level,
                        updated_at = CURRENT_TIMESTAMP
                    """,
                    (user_id, expertise_level),
                )
                conn.commit()

    async def create_session(self, session_id: str, user_id: str) -> None:
        """
        Create a new conversation session.

        Args:
            session_id: Unique session identifier
            user_id: User identifier
        """
        with self.get_connection() as conn:
            with conn.cursor() as cur:
                cur.execute(
                    """
                    INSERT INTO conversation_sessions (session_id, user_id)
                    VALUES (%s, %s)
                    """,
                    (session_id, user_id),
                )
                conn.commit()

    async def get_session(self, session_id: str) -> dict | None:
        """
        Get session by ID.

        Args:
            session_id: Session identifier

        Returns:
            Session dict or None if not found
        """
        with self.get_connection() as conn:
            with conn.cursor() as cur:
                cur.execute(
                    "SELECT * FROM conversation_sessions WHERE session_id = %s",
                    (session_id,),
                )
                return cur.fetchone()

    async def update_session_activity(self, session_id: str) -> None:
        """
        Update session's last activity timestamp.

        Args:
            session_id: Session identifier
        """
        with self.get_connection() as conn:
            with conn.cursor() as cur:
                cur.execute(
                    """
                    UPDATE conversation_sessions
                    SET last_activity = CURRENT_TIMESTAMP
                    WHERE session_id = %s
                    """,
                    (session_id,),
                )
                conn.commit()

    async def add_message(
        self,
        session_id: str,
        role: str,
        content: str,
    ) -> None:
        """
        Add a message to a conversation session.

        Args:
            session_id: Session identifier
            role: Message role (user/assistant/system)
            content: Message content
        """
        with self.get_connection() as conn:
            with conn.cursor() as cur:
                cur.execute(
                    """
                    INSERT INTO conversation_messages (session_id, role, content)
                    VALUES (%s, %s, %s)
                    """,
                    (session_id, role, content),
                )
                conn.commit()

    async def get_session_messages(
        self,
        session_id: str,
        limit: int | None = None,
    ) -> list[dict]:
        """
        Get messages for a session.

        Args:
            session_id: Session identifier
            limit: Optional limit on number of messages

        Returns:
            List of message dicts
        """
        with self.get_connection() as conn:
            with conn.cursor() as cur:
                query = """
                    SELECT role, content, timestamp
                    FROM conversation_messages
                    WHERE session_id = %s
                    ORDER BY timestamp ASC
                """
                if limit:
                    query += f" LIMIT {limit}"

                cur.execute(query, (session_id,))
                return cur.fetchall()

    async def deactivate_expired_sessions(self, timeout_minutes: int) -> None:
        """
        Deactivate sessions that have been inactive for too long.

        Args:
            timeout_minutes: Timeout threshold in minutes
        """
        with self.get_connection() as conn:
            with conn.cursor() as cur:
                cur.execute(
                    """
                    UPDATE conversation_sessions
                    SET is_active = FALSE
                    WHERE is_active = TRUE
                    AND last_activity < NOW() - INTERVAL '%s minutes'
                    """,
                    (timeout_minutes,),
                )
                conn.commit()


# Global Postgres service instance
postgres_service = PostgresService()
