import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

# Test with the actual file
file_path = Path("../../docs/chapter-3-isaac-ai-brain/bipedal-path-planning.md")

print(f"Reading: {file_path}")
with open(file_path, "r", encoding="utf-8") as f:
    content = f.read()

print(f"File size: {len(content)} chars")

# Manually test the chunking logic
lines = content.split('\n')
print(f"Total lines: {len(lines)}")

import re
current_section = []
chunks_count = 0

for i, line in enumerate(lines):
    if i % 100 == 0:
        print(f"Processing line {i}/{len(lines)}", flush=True)
    
    if line.startswith('#'):
        header_match = re.match(r'^(#{1,6})\s+(.+)$', line)
        if header_match:
            if current_section:
                chunks_count += 1
            current_section = [line]
    else:
        current_section.append(line)

print(f"\n✓ Completed! Found {chunks_count} sections")
