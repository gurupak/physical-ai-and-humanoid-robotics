import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

from app.utils import text_chunker

# Test with the actual file that seems to hang
file_path = Path("../../docs/chapter-3-isaac-ai-brain/bipedal-path-planning.md")

if not file_path.exists():
    print(f"File not found: {file_path}")
    sys.exit(1)

print(f"Reading: {file_path}")
with open(file_path, "r", encoding="utf-8") as f:
    content = f.read()

print(f"File size: {len(content)} chars")
print(f"Lines: {content.count(chr(10))}")

print("\nStarting chunking...")
chunks = text_chunker.chunk_by_sections(content)
print(f"✓ Created {len(chunks)} chunks successfully!")

for i, (chunk_text, metadata) in enumerate(chunks[:3]):
    print(f"\nChunk {i+1}: {metadata['title'][:50] if metadata['title'] else 'No title'}")
