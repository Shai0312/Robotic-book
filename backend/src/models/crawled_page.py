from dataclasses import dataclass
from typing import Dict, Optional
from datetime import datetime


@dataclass
class CrawledPage:
    """
    A web page from a Docusaurus site that has been successfully crawled and processed.

    Attributes:
        url: The URL of the page
        title: The title of the page
        content: The extracted text content
        html: The original HTML content
        status: Status of the crawl operation
        crawled_at: Timestamp when the page was crawled
        processed_content: Cleaned text content after processing
    """

    url: str
    title: str
    content: str
    html: str
    status: str
    crawled_at: datetime = None
    processed_content: str = ""

    def __post_init__(self):
        if self.crawled_at is None:
            self.crawled_at = datetime.now()

    def to_dict(self) -> Dict:
        """Convert the CrawledPage to a dictionary representation."""
        return {
            "url": self.url,
            "title": self.title,
            "content": self.content,
            "html": self.html,
            "status": self.status,
            "crawled_at": self.crawled_at.isoformat() if self.crawled_at else None,
            "processed_content": self.processed_content
        }

    @classmethod
    def from_dict(cls, data: Dict) -> 'CrawledPage':
        """Create a CrawledPage from a dictionary representation."""
        page = cls(
            url=data["url"],
            title=data["title"],
            content=data["content"],
            html=data["html"],
            status=data["status"],
            processed_content=data.get("processed_content", "")
        )
        if "crawled_at" in data and data["crawled_at"]:
            page.crawled_at = datetime.fromisoformat(data["crawled_at"])
        return page