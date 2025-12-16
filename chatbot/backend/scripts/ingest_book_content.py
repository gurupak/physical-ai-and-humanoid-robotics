"""Script to ingest book content into Qdrant vector database."""

import asyncio
import os
import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

# Set UTF-8 encoding for Windows console
if sys.platform == "win32":
    sys.stdout.reconfigure(encoding="utf-8")

from app.db import qdrant_service
from app.services import embedding_service
from app.utils import text_chunker


async def read_markdown_files(docs_dir: str) -> list[tuple[str, str]]:
    """
    Read all markdown files from the docs directory.

    Args:
        docs_dir: Path to docs directory

    Returns:
        List of (file_path, content) tuples
    """
    docs_path = Path(docs_dir)
    markdown_files = []

    if not docs_path.exists():
        print(f"[ERROR] Directory not found: {docs_dir}")
        return []

    # Find all .md and .mdx files
    for ext in ["*.md", "*.mdx"]:
        for file_path in docs_path.rglob(ext):
            # Skip non-content files
            if any(skip in str(file_path) for skip in ["README", "node_modules", ".git"]):
                continue

            try:
                with open(file_path, "r", encoding="utf-8") as f:
                    content = f.read()
                    markdown_files.append((str(file_path), content))
                    print(f"[OK] Read: {file_path.relative_to(docs_path)}")
            except Exception as e:
                print(f"[ERROR] Error reading {file_path}: {e}")

    return markdown_files


async def ingest_content(docs_dir: str) -> None:
    """
    Ingest book content into Qdrant.

    Args:
        docs_dir: Path to docs directory containing markdown files
    """
    print("=" * 60)
    print("Starting book content ingestion...")
    print("=" * 60)
    print()

    # Ensure Qdrant collection exists
    print("[STEP 1] Ensuring Qdrant collection exists...")
    await qdrant_service.ensure_collection_exists()
    print("[OK] Collection ready")
    print()

    # Read markdown files
    print(f"[STEP 2] Reading markdown files from: {docs_dir}")
    files = await read_markdown_files(docs_dir)

    if not files:
        print("[ERROR] No markdown files found!")
        return

    print(f"[OK] Found {len(files)} files")
    print()

    # Process each file
    all_chunks = []
    all_metadata = []

    print("[STEP 3] Processing files and creating chunks...")
    for idx, (file_path, content) in enumerate(files, 1):
        print(f"  [{idx}/{len(files)}] Processing: {Path(file_path).name}", end="", flush=True)

        try:
            # Chunk the content
            print(f" (content size: {len(content)} chars)...", end="", flush=True)
            chunks_with_metadata = text_chunker.chunk_by_sections(content)
            print(f" chunked...", end="", flush=True)

            for chunk_text, metadata in chunks_with_metadata:
                all_chunks.append(chunk_text)
                all_metadata.append(
                    {
                        **metadata,
                        "source_file": str(Path(file_path).relative_to(docs_dir)),
                    }
                )

            print(f" -> {len(chunks_with_metadata)} chunks", flush=True)
        except Exception as e:
            print(f" [ERROR: {e}]", flush=True)

    print(f"[OK] Total chunks created: {len(all_chunks)}")
    print()

    # Generate embeddings in batches
    print("[STEP 4] Generating embeddings...")
    batch_size = 100
    all_embeddings = []
    total_batches = (len(all_chunks) + batch_size - 1) // batch_size

    for i in range(0, len(all_chunks), batch_size):
        batch = all_chunks[i : i + batch_size]
        batch_num = i // batch_size + 1
        print(
            f"  [{batch_num}/{total_batches}] Processing batch... ({len(batch)} chunks)",
            end="",
            flush=True,
        )

        batch_embeddings = await embedding_service.generate_embeddings_batch(batch)
        all_embeddings.extend(batch_embeddings)
        print(f" ✓", flush=True)

    print(f"[OK] Generated {len(all_embeddings)} embeddings")
    print()

    # Upsert to Qdrant in batches to avoid connection issues
    print("[STEP 5] Uploading to Qdrant...")
    upload_batch_size = 100  # Reduced batch size for Qdrant Cloud free tier
    total_upload_batches = (len(all_chunks) + upload_batch_size - 1) // upload_batch_size

    for i in range(0, len(all_chunks), upload_batch_size):
        batch_chunks = all_chunks[i : i + upload_batch_size]
        batch_embeddings = all_embeddings[i : i + upload_batch_size]
        batch_metadata = all_metadata[i : i + upload_batch_size]

        batch_num = i // upload_batch_size + 1
        print(
            f"  [{batch_num}/{total_upload_batches}] Uploading batch... ({len(batch_chunks)} chunks)",
            end="",
            flush=True,
        )

        # Retry logic for network issues
        max_retries = 3
        for retry in range(max_retries):
            try:
                await qdrant_service.upsert_chunks(
                    chunks=batch_chunks,
                    embeddings=batch_embeddings,
                    metadata=batch_metadata,
                    start_id=i,
                )
                print(f" ✓", flush=True)
                break
            except Exception as e:
                if retry < max_retries - 1:
                    print(f" [Retry {retry + 1}/{max_retries - 1}]", end="", flush=True)
                    await asyncio.sleep(2)  # Wait before retry
                else:
                    print(f" ✗ Failed: {e}", flush=True)
                    raise

    print("[OK] Upload complete")
    print()

    # Get collection info
    info = await qdrant_service.get_collection_info()
    if info:
        print("=" * 60)
        print("Collection Info:")
        print(f"  Points: {info['points_count']}")
        print(f"  Status: {info['status']}")
        print("=" * 60)

    print()
    print("[SUCCESS] Ingestion complete!")


async def main() -> None:
    """Main entry point."""
    # Get docs directory from command line or use default
    if len(sys.argv) > 1:
        docs_dir = sys.argv[1]
    else:
        # Default to project root /docs directory
        project_root = Path(__file__).parent.parent.parent.parent
        docs_dir = str(project_root / "docs")

    await ingest_content(docs_dir)


if __name__ == "__main__":
    asyncio.run(main())
