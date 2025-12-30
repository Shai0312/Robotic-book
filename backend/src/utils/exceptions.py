"""
Custom exception classes for the RAG knowledge ingestion pipeline.
"""


class RAGException(Exception):
    """Base exception for RAG-related errors."""
    pass


class CrawlerException(RAGException):
    """Exception raised for errors during crawling operations."""
    pass


class TextProcessingException(RAGException):
    """Exception raised for errors during text processing operations."""
    pass


class EmbeddingException(RAGException):
    """Exception raised for errors during embedding operations."""
    pass


class StorageException(RAGException):
    """Exception raised for errors during storage operations."""
    pass


class ConfigurationException(RAGException):
    """Exception raised for configuration-related errors."""
    pass


class URLValidationException(RAGException):
    """Exception raised when URL validation fails."""
    pass


class ContentExtractionException(RAGException):
    """Exception raised when content extraction fails."""
    pass


class RateLimitException(RAGException):
    """Exception raised when API rate limits are exceeded."""
    pass