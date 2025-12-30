from dataclasses import dataclass
from typing import Dict, List
from datetime import datetime


@dataclass
class EmbeddingRecord:
    """
    A record containing the embedding vector and associated metadata.

    Attributes:
        id: Unique identifier for the record
        vector: The embedding vector
        payload: Metadata associated with the vector
        created_at: Timestamp when the record was created
    """

    id: str
    vector: List[float]
    payload: Dict[str, any]
    created_at: datetime = None

    def __post_init__(self):
        if self.created_at is None:
            self.created_at = datetime.now()

    def to_dict(self) -> Dict:
        """Convert the EmbeddingRecord to a dictionary representation."""
        return {
            "id": self.id,
            "vector": self.vector,
            "payload": self.payload,
            "created_at": self.created_at.isoformat() if self.created_at else None
        }

    @classmethod
    def from_dict(cls, data: Dict) -> 'EmbeddingRecord':
        """Create an EmbeddingRecord from a dictionary representation."""
        record = cls(
            id=data["id"],
            vector=data["vector"],
            payload=data["payload"]
        )
        if "created_at" in data and data["created_at"]:
            record.created_at = datetime.fromisoformat(data["created_at"])
        return record