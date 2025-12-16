"""Database migration script to add previous_response_id column."""

import os

import psycopg2
from dotenv import load_dotenv

# Load environment variables
load_dotenv()


def migrate():
    """Add previous_response_id column to conversation_sessions table."""
    database_url = os.getenv("DATABASE_URL")

    if not database_url:
        print("ERROR: DATABASE_URL not found in environment variables")
        return

    print(f"Connecting to database...")

    try:
        conn = psycopg2.connect(database_url)
        cur = conn.cursor()

        print("Adding previous_response_id column...")
        cur.execute("""
            ALTER TABLE conversation_sessions
            ADD COLUMN IF NOT EXISTS previous_response_id TEXT;
        """)

        conn.commit()
        print("✓ Migration successful! Column 'previous_response_id' added.")

        cur.close()
        conn.close()

    except Exception as e:
        print(f"ERROR: Migration failed - {e}")


if __name__ == "__main__":
    migrate()
