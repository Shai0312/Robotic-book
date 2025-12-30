import os
from dotenv import load_dotenv
from typing import Optional

# Load environment variables from .env file
load_dotenv()

class Settings:
    """Configuration settings for the RAG knowledge ingestion pipeline."""

    # Cohere Configuration
    COHERE_API_KEY: str = os.getenv("COHERE_API_KEY", "")

    # Qdrant Configuration
    QDRANT_URL: str = os.getenv("QDRANT_URL", "http://localhost:6333")
    QDRANT_API_KEY: Optional[str] = os.getenv("QDRANT_API_KEY")
    QDRANT_COLLECTION_NAME: str = os.getenv("QDRANT_COLLECTION_NAME", "documents")

    # Crawler Configuration
    CRAWLER_DELAY: float = float(os.getenv("CRAWLER_DELAY", "1.0"))
    CRAWLER_MAX_PAGES: int = int(os.getenv("CRAWLER_MAX_PAGES", "1000"))
    CRAWLER_TIMEOUT: int = int(os.getenv("CRAWLER_TIMEOUT", "30"))

    # Text Processing Configuration
    CHUNK_SIZE: int = int(os.getenv("CHUNK_SIZE", "512"))
    CHUNK_OVERLAP: int = int(os.getenv("CHUNK_OVERLAP", "50"))

    # Logging Configuration
    LOG_LEVEL: str = os.getenv("LOG_LEVEL", "INFO")

    def __init__(self):
        """Validate required settings."""
        if not self.COHERE_API_KEY:
            raise ValueError("COHERE_API_KEY environment variable must be set")
        if not self.QDRANT_URL:
            raise ValueError("QDRANT_URL environment variable must be set")

    @property
    def has_valid_config(self) -> bool:
        """Check if the configuration is valid."""
        return bool(self.COHERE_API_KEY and self.QDRANT_URL)


# Global settings instance
settings = Settings()