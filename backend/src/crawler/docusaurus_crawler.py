import time
from typing import List, Set, Optional
from urllib.parse import urljoin, urlparse
from src.utils.logger import get_logger
from src.utils.exceptions import CrawlerException
from src.utils.helpers import extract_links_from_html, is_same_domain, check_url_accessibility
from src.models.crawled_page import CrawledPage
from src.crawler.url_fetcher import URLFetcher
from src.config.settings import settings

logger = get_logger(__name__)


class DocusaurusCrawler:
    """
    Crawls Docusaurus documentation sites to extract content.
    """

    def __init__(self):
        self.fetcher = URLFetcher()
        self.visited_urls: Set[str] = set()
        self.pending_urls: List[str] = []
        self.crawled_pages: List[CrawledPage] = []

    def _get_sitemap_urls(self, base_url: str) -> List[str]:
        """
        Try to get URLs from the sitemap.xml file.

        Args:
            base_url: The base URL of the site

        Returns:
            List of URLs from the sitemap, or empty list if sitemap not found
        """
        sitemap_url = urljoin(base_url, 'sitemap.xml')

        try:
            if check_url_accessibility(sitemap_url):
                content = self.fetcher.fetch_with_retry(sitemap_url)
                if content:
                    # Simple extraction of URLs from sitemap XML
                    import re
                    # Look for <loc> tags containing URLs
                    url_pattern = r'<loc>(https?://[^<]+)</loc>'
                    found_urls = re.findall(url_pattern, content, re.IGNORECASE)

                    # Filter URLs to only include those from the same domain
                    valid_urls = []
                    for url in found_urls:
                        if is_same_domain(base_url, url):
                            # Check if the URL is accessible before adding to the list
                            # This helps avoid 404s when crawling
                            if check_url_accessibility(url):
                                valid_urls.append(url)
                            else:
                                logger.debug(f"Skipping inaccessible URL from sitemap: {url}")

                    logger.info(f"Found {len(valid_urls)} accessible URLs from sitemap: {sitemap_url}")
                    return valid_urls
        except Exception as e:
            logger.warning(f"Could not fetch or parse sitemap {sitemap_url}: {str(e)}")

        return []

    def crawl_site(self, start_url: str) -> List[CrawledPage]:
        """
        Crawl a Docusaurus site starting from the given URL.

        Args:
            start_url: The starting URL to crawl from

        Returns:
            List of CrawledPage objects containing the extracted content
        """
        logger.info(f"Starting crawl of Docusaurus site: {start_url}")
        self.visited_urls = set()
        self.pending_urls = [start_url]
        self.crawled_pages = []

        base_domain = urlparse(start_url).netloc

        # First, try to get URLs from sitemap to supplement the crawl
        sitemap_urls = self._get_sitemap_urls(start_url)
        for url in sitemap_urls:
            if url not in self.pending_urls and url not in self.visited_urls:
                self.pending_urls.append(url)

        while self.pending_urls and len(self.visited_urls) < settings.CRAWLER_MAX_PAGES:
            current_url = self.pending_urls.pop(0)

            if current_url in self.visited_urls:
                continue

            self.visited_urls.add(current_url)
            logger.info(f"Crawling page ({len(self.visited_urls)}/{settings.CRAWLER_MAX_PAGES}): {current_url}")

            try:
                # Check URL accessibility first to avoid unnecessary requests
                if not check_url_accessibility(current_url):
                    logger.warning(f"Skipping inaccessible URL: {current_url}")
                    continue

                # Fetch the content of the current page
                content = self.fetcher.fetch_with_retry(current_url)
                if content is None:
                    logger.warning(f"Could not fetch content for URL: {current_url}")
                    continue

                # Create a CrawledPage object
                crawled_page = CrawledPage(
                    url=current_url,
                    title=self._extract_title(content) or "Untitled",
                    content=content,
                    html=content,
                    status="success"
                )

                self.crawled_pages.append(crawled_page)

                # Extract links from the page content and add to pending URLs
                links = extract_links_from_html(content, start_url)
                for link in links:
                    if (link not in self.visited_urls and
                        link not in self.pending_urls and
                        is_same_domain(start_url, link)):

                        # Check if the link is accessible before adding to pending URLs
                        # This helps avoid crawling URLs that return 404
                        if (len(self.visited_urls) + len(self.pending_urls) < settings.CRAWLER_MAX_PAGES and
                            check_url_accessibility(link)):
                            self.pending_urls.append(link)
                        else:
                            logger.debug(f"Skipping URL (not accessible or limit reached): {link}")

                # Be respectful to the server by adding a delay
                time.sleep(settings.CRAWLER_DELAY)

            except Exception as e:
                logger.error(f"Error crawling URL {current_url}: {str(e)}")
                # Add a failed CrawledPage entry
                failed_page = CrawledPage(
                    url=current_url,
                    title="Error",
                    content="",
                    html="",
                    status=f"error: {str(e)}"
                )
                self.crawled_pages.append(failed_page)

        logger.info(f"Crawling completed. Total pages crawled: {len(self.crawled_pages)}")
        return self.crawled_pages

    def _extract_title(self, html_content: str) -> Optional[str]:
        """
        Extract the title from HTML content.

        Args:
            html_content: HTML content to extract title from

        Returns:
            The extracted title, or None if not found
        """
        from bs4 import BeautifulSoup

        try:
            soup = BeautifulSoup(html_content, 'html.parser')
            title_tag = soup.find('title')
            if title_tag:
                return title_tag.get_text().strip()
            return None
        except Exception as e:
            logger.error(f"Error extracting title from HTML: {str(e)}")
            return None

    def get_crawled_pages(self) -> List[CrawledPage]:
        """
        Get the list of crawled pages.

        Returns:
            List of CrawledPage objects
        """
        return self.crawled_pages


def crawl_docusaurus_site(start_url: str) -> List[CrawledPage]:
    """
    Convenience function to crawl a Docusaurus site.

    Args:
        start_url: The starting URL to crawl from

    Returns:
        List of CrawledPage objects containing the extracted content
    """
    crawler = DocusaurusCrawler()
    return crawler.crawl_site(start_url)