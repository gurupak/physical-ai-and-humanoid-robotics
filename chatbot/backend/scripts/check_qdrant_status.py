"""Quick script to check Qdrant collection status."""

import asyncio
import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.db import qdrant_service


async def main() -> None:
    """Check Qdrant collection status."""
    print("🔍 Checking Qdrant collection status...\n")

    info = await qdrant_service.get_collection_info()

    if info:
        print("✅ Collection exists!")
        print(f"📊 Points count: {info['points_count']}")
        print(f"📊 Status: {info['status']}")
    else:
        print("❌ Collection not found or connection failed")


if __name__ == "__main__":
    asyncio.run(main())
