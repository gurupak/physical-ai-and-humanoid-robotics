import asyncio
from qdrant_client import QdrantClient

async def test():
    client = QdrantClient(
        url="https://dff0a2ac-5a2c-4a5d-80c7-665cbd1e0cb0.us-east4-0.gcp.cloud.qdrant.io",
        api_key="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.fMvz8xJgQMjRU--jeXXh5fr1uClE2f6iDGVGH3t8zyU",
        timeout=120,
    )
    
    try:
        collections = client.get_collections()
        print(f"Collections: {[c.name for c in collections.collections]}")
        
        if collections.collections:
            for col in collections.collections:
                info = client.get_collection(col.name)
                print(f"\nCollection: {col.name}")
                print(f"  Points: {info.points_count}")
                print(f"  Vectors: {info.vectors_count}")
    except Exception as e:
        print(f"Error: {e}")

asyncio.run(test())
