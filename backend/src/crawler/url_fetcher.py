import requests
from typing import Optional
from src.utils.logger import get_logger
from src.utils.exceptions import CrawlerException
from src.config.settings import settings

logger = get_logger(__name__)


class URLFetcher:
    """
    Fetches URLs and returns their content.
    """

    def __init__(self):
        self.session = requests.Session()
        # Set a user agent to be respectful to websites
        self.session.headers.update({
            'User-Agent': 'RAG-Knowledge-Ingestion-Bot/1.0'
        })

    def fetch_url(self, url: str) -> Optional[str]:
        """
        Fetch the content of a URL.

        Args:
            url: The URL to fetch

        Returns:
            The content of the URL as a string, or None if the fetch failed
        """
        try:
            logger.info(f"Fetching URL: {url}")

            response = self.session.get(
                url,
                timeout=settings.CRAWLER_TIMEOUT,
                allow_redirects=True
            )

            # Check if the request was successful
            if response.status_code == 200:
                logger.info(f"Successfully fetched URL: {url}")
                return response.text
            else:
                logger.warning(f"Failed to fetch URL: {url}, status code: {response.status_code}")
                return None

        except requests.exceptions.RequestException as e:
            logger.error(f"Error fetching URL {url}: {str(e)}")
            raise CrawlerException(f"Error fetching URL {url}: {str(e)}")

    def fetch_with_retry(self, url: str, max_retries: int = 3) -> Optional[str]:
        """
        Fetch the content of a URL with retry logic.

        Args:
            url: The URL to fetch
            max_retries: Maximum number of retry attempts

        Returns:
            The content of the URL as a string, or None if all retries failed
        """
        for attempt in range(max_retries):
            try:
                content = self.fetch_url(url)
                if content is not None:
                    return content
                # Wait before retrying (exponential backoff)
                import time
                time.sleep(2 ** attempt)
            except CrawlerException:
                if attempt == max_retries - 1:
                    # If this was the last attempt, re-raise the exception
                    raise
                # Wait before retrying
                import time
                time.sleep(2 ** attempt)

        logger.error(f"Failed to fetch URL after {max_retries} attempts: {url}")
        return None


def fetch_url_content(url: str) -> Optional[str]:
    """
    Convenience function to fetch URL content.

    Args:
        url: The URL to fetch

    Returns:
        The content of the URL as a string, or None if the fetch failed
    """
    fetcher = URLFetcher()
    return fetcher.fetch_with_retry(url)