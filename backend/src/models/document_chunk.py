from dataclasses import dataclass
from typing import Dict, List, Optional
from datetime import datetime


@dataclass
class DocumentChunk:
    """
    A segment of cleaned text content extracted from documentation pages.

    Attributes:
        id: Unique identifier for the chunk
        content: The actual text content of the chunk
        source_url: URL of the original documentation page
        title: Title of the original documentation page
        metadata: Additional metadata about the chunk
        embedding: Vector representation of the content
        created_at: Timestamp when the chunk was created
    """

    id: str
    content: str
    source_url: str
    title: str
    metadata: Dict[str, any] = None
    embedding: Optional[List[float]] = None
    created_at: datetime = None

    def __post_init__(self):
        if self.metadata is None:
            self.metadata = {}
        if self.created_at is None:
            self.created_at = datetime.now()

    def to_dict(self) -> Dict:
        """Convert the DocumentChunk to a dictionary representation."""
        return {
            "id": self.id,
            "content": self.content,
            "source_url": self.source_url,
            "title": self.title,
            "metadata": self.metadata,
            "embedding": self.embedding,
            "created_at": self.created_at.isoformat() if self.created_at else None
        }

    @classmethod
    def from_dict(cls, data: Dict) -> 'DocumentChunk':
        """Create a DocumentChunk from a dictionary representation."""
        chunk = cls(
            id=data["id"],
            content=data["content"],
            source_url=data["source_url"],
            title=data["title"],
            metadata=data.get("metadata", {}),
            embedding=data.get("embedding")
        )
        if "created_at" in data and data["created_at"]:
            chunk.created_at = datetime.fromisoformat(data["created_at"])
        return chunk