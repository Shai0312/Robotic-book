from dataclasses import dataclass
from typing import Dict
from datetime import datetime


@dataclass
class CrawlSession:
    """
    A session representing a single crawling operation.

    Attributes:
        id: Unique identifier for the session
        target_url: The base URL that was crawled
        status: Current status of the session
        total_pages_found: Number of pages discovered during crawling
        successful_pages: Number of pages successfully processed
        start_time: When the session started
        end_time: When the session ended
        settings: Crawl settings used for this session
    """

    id: str
    target_url: str
    status: str
    total_pages_found: int
    successful_pages: int
    start_time: datetime = None
    end_time: datetime = None
    settings: Dict[str, any] = None

    def __post_init__(self):
        if self.start_time is None:
            self.start_time = datetime.now()
        if self.settings is None:
            self.settings = {}

    def to_dict(self) -> Dict:
        """Convert the CrawlSession to a dictionary representation."""
        return {
            "id": self.id,
            "target_url": self.target_url,
            "status": self.status,
            "total_pages_found": self.total_pages_found,
            "successful_pages": self.successful_pages,
            "start_time": self.start_time.isoformat() if self.start_time else None,
            "end_time": self.end_time.isoformat() if self.end_time else None,
            "settings": self.settings
        }

    @classmethod
    def from_dict(cls, data: Dict) -> 'CrawlSession':
        """Create a CrawlSession from a dictionary representation."""
        session = cls(
            id=data["id"],
            target_url=data["target_url"],
            status=data["status"],
            total_pages_found=data["total_pages_found"],
            successful_pages=data["successful_pages"],
            settings=data.get("settings", {})
        )
        if "start_time" in data and data["start_time"]:
            session.start_time = datetime.fromisoformat(data["start_time"])
        if "end_time" in data and data["end_time"]:
            session.end_time = datetime.fromisoformat(data["end_time"])
        return session