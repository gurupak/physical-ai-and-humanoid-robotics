"""Text chunking utilities for semantic content splitting."""

import re
from typing import List, Tuple

from app.core import settings


class TextChunker:
    """Semantic text chunking for book content."""

    def __init__(
        self,
        chunk_size: int = settings.chunk_size,
        chunk_overlap: int = settings.chunk_overlap,
    ) -> None:
        """
        Initialize text chunker.

        Args:
            chunk_size: Maximum characters per chunk
            chunk_overlap: Character overlap between chunks
        """
        self.chunk_size = chunk_size
        self.chunk_overlap = chunk_overlap

    def chunk_by_sections(self, text: str) -> List[Tuple[str, dict]]:
        """
        Chunk text by markdown sections/subsections with metadata.

        Args:
            text: Input text content

        Returns:
            List of (chunk_text, metadata) tuples
        """
        chunks: List[Tuple[str, dict]] = []

        # Split by markdown headers (# , ## , ### , etc.)
        # Use a simpler, non-capturing split to avoid catastrophic backtracking
        lines = text.split("\n")
        current_section = []
        current_metadata = {"level": 0, "title": ""}

        for line in lines:
            # Check if this is a header (more efficient than regex on large text)
            if line.startswith("#"):
                header_match = re.match(r"^(#{1,6})\s+(.+)$", line)

                if header_match:
                    # Save previous section if it exists
                    if current_section:
                        section_text = "\n".join(current_section)
                        if section_text.strip():
                            section_chunks = self._split_large_section(
                                section_text,
                                current_metadata,
                            )
                            chunks.extend(section_chunks)

                    # Start new section
                    level = len(header_match.group(1))
                    title = header_match.group(2).strip()
                    current_metadata = {"level": level, "title": title}
                    current_section = [line]
                else:
                    current_section.append(line)
            else:
                current_section.append(line)

        # Don't forget the last section
        if current_section:
            section_text = "\n".join(current_section)
            if section_text.strip():
                section_chunks = self._split_large_section(section_text, current_metadata)
                chunks.extend(section_chunks)

        return chunks

    def _split_large_section(
        self,
        text: str,
        metadata: dict,
    ) -> List[Tuple[str, dict]]:
        """
        Split a large section into smaller chunks if needed.

        Args:
            text: Section text
            metadata: Section metadata

        Returns:
            List of (chunk_text, metadata) tuples
        """
        if len(text) <= self.chunk_size:
            return [(text.strip(), metadata)]

        chunks: List[Tuple[str, dict]] = []
        start = 0
        max_iterations = 1000  # Safety limit to prevent infinite loops

        iteration = 0
        while start < len(text) and iteration < max_iterations:
            iteration += 1
            end = start + self.chunk_size

            # Try to break at sentence boundaries
            if end < len(text):
                # Look for sentence endings
                sentence_end = self._find_sentence_boundary(text, start, end)
                if sentence_end > start:
                    end = sentence_end

            chunk = text[start:end].strip()
            if chunk:
                chunk_metadata = {
                    **metadata,
                    "chunk_index": len(chunks),
                    "is_partial": True,
                }
                chunks.append((chunk, chunk_metadata))

            # Ensure we're making progress
            if end <= start:
                start += self.chunk_size
            else:
                start = end - self.chunk_overlap

        return chunks

    @staticmethod
    def _find_sentence_boundary(text: str, start: int, end: int) -> int:
        """
        Find the nearest sentence boundary before the end position.

        Args:
            text: Full text
            start: Start position
            end: Desired end position

        Returns:
            Actual end position at sentence boundary
        """
        # Look for sentence endings: . ! ? followed by space or newline
        search_text = text[start:end]

        # Use rfind instead of regex for better performance
        last_pos = -1
        for char in [". ", "! ", "? ", ".\n", "!\n", "?\n"]:
            pos = search_text.rfind(char)
            if pos > last_pos:
                last_pos = pos

        if last_pos > 0:
            return start + last_pos + 2  # +2 to include the punctuation and space

        # If no sentence boundary found, look for paragraph breaks
        para_pos = search_text.rfind("\n\n")
        if para_pos > 0:
            return start + para_pos + 2

        # Fall back to original end
        return end

    def chunk_by_fixed_size(self, text: str) -> List[str]:
        """
        Simple fixed-size chunking with overlap (fallback method).

        Args:
            text: Input text

        Returns:
            List of text chunks
        """
        chunks: List[str] = []
        start = 0

        while start < len(text):
            end = start + self.chunk_size
            chunk = text[start:end].strip()
            if chunk:
                chunks.append(chunk)
            start = end - self.chunk_overlap

        return chunks


# Global chunker instance
text_chunker = TextChunker()
