"""Query analysis utilities for determining query type and complexity."""

import re
from typing import Tuple

from app.models.query import QueryComplexity, QueryType


class QueryAnalyzer:
    """Analyze user queries to determine type and complexity."""

    @staticmethod
    def determine_query_type(query: str, selected_text: str | None) -> QueryType:
        """
        Determine if query is general or based on selected text.

        Args:
            query: User query
            selected_text: Selected text from book (if any)

        Returns:
            QueryType enum
        """
        if selected_text:
            return QueryType.SELECTED_TEXT
        return QueryType.GENERAL

    @staticmethod
    def analyze_complexity(query: str) -> QueryComplexity:
        """
        Analyze query complexity based on heuristics.

        Args:
            query: User query

        Returns:
            QueryComplexity enum
        """
        # Simple heuristics for complexity detection
        word_count = len(query.split())
        has_multiple_questions = query.count("?") > 1
        has_compound_sentences = bool(
            re.search(r"\b(and|or|but|however|moreover|furthermore)\b", query, re.IGNORECASE),
        )
        has_technical_terms = bool(
            re.search(
                r"\b(explain|compare|analyze|evaluate|discuss|"
                r"relationship|difference|implementation|architecture)\b",
                query,
                re.IGNORECASE,
            ),
        )

        # Determine complexity
        complexity_score = 0

        if word_count > 15:
            complexity_score += 2
        elif word_count > 8:
            complexity_score += 1

        if has_multiple_questions:
            complexity_score += 2

        if has_compound_sentences:
            complexity_score += 1

        if has_technical_terms:
            complexity_score += 1

        # Map score to complexity level
        if complexity_score >= 4:
            return QueryComplexity.COMPLEX
        elif complexity_score >= 2:
            return QueryComplexity.MODERATE
        else:
            return QueryComplexity.SIMPLE

    @staticmethod
    def calculate_chunks_to_retrieve(
        complexity: QueryComplexity,
        expertise_level: str,
        min_chunks: int = 3,
        max_chunks: int = 10,
    ) -> int:
        """
        Calculate optimal number of chunks to retrieve based on complexity and expertise.

        Args:
            complexity: Query complexity level
            expertise_level: User's expertise level
            min_chunks: Minimum chunks to retrieve
            max_chunks: Maximum chunks to retrieve

        Returns:
            Number of chunks to retrieve
        """
        # Base chunks based on complexity
        complexity_chunks = {
            QueryComplexity.SIMPLE: 3,
            QueryComplexity.MODERATE: 5,
            QueryComplexity.COMPLEX: 7,
        }

        base_chunks = complexity_chunks.get(complexity, 5)

        # Expertise multiplier
        expertise_multiplier = {
            "beginner": 1.2,  # More context for beginners
            "intermediate": 1.0,
            "advanced": 0.8,  # Less context for advanced users
        }

        multiplier = expertise_multiplier.get(expertise_level.lower(), 1.0)
        calculated_chunks = int(base_chunks * multiplier)

        # Clamp to min/max range
        return max(min_chunks, min(calculated_chunks, max_chunks))


# Global analyzer instance
query_analyzer = QueryAnalyzer()
