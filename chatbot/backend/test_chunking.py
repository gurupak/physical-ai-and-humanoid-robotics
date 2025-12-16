import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

from app.utils import text_chunker

# Test with a simple markdown
test_content = """
# Introduction

This is a test document.

## Section 1

Some content here.

## Section 2

More content here.
"""

print("Testing chunking...")
print(f"Input length: {len(test_content)} chars")

chunks = text_chunker.chunk_by_sections(test_content)
print(f"Number of chunks: {len(chunks)}")

for i, (chunk_text, metadata) in enumerate(chunks):
    print(f"\nChunk {i+1}:")
    print(f"  Metadata: {metadata}")
    print(f"  Text length: {len(chunk_text)} chars")
    print(f"  First 100 chars: {chunk_text[:100]}")

print("\n✓ Chunking test complete!")
