from bs4 import BeautifulSoup
from typing import List, Tuple
import re
from src.utils.logger import get_logger
from src.utils.exceptions import TextProcessingException
from src.models.crawled_page import CrawledPage

logger = get_logger(__name__)


class TextCleaner:
    """
    Cleans HTML content and extracts meaningful text from web pages.
    """

    def __init__(self):
        pass

    def clean_html_content(self, html_content: str) -> str:
        """
        Clean HTML content by removing tags and extracting text.
        Optimized for Docusaurus documentation sites to preserve all meaningful content.

        Args:
            html_content: Raw HTML content

        Returns:
            Clean text content
        """
        try:
            soup = BeautifulSoup(html_content, 'html.parser')

            # Remove only the most problematic elements, preserve documentation content
            for script in soup(["script", "style"]):
                script.decompose()

            # Remove elements that are definitely not content, but be more selective for documentation
            # Keep main content containers that might have important info in documentation sites
            for element in soup(["noscript", "meta", "link"]):
                element.decompose()

            # Get text content with proper line breaks to preserve structure
            text = soup.get_text(separator='\n')

            # Clean up the text while preserving paragraph structure
            lines = (line.strip() for line in text.splitlines())
            # Join non-empty lines while preserving document structure
            text = '\n'.join(line for line in lines if line)

            return text
        except Exception as e:
            logger.error(f"Error cleaning HTML content: {str(e)}")
            raise TextProcessingException(f"Error cleaning HTML content: {str(e)}")

    def extract_content_from_page(self, crawled_page: CrawledPage) -> str:
        """
        Extract clean content from a crawled page.

        Args:
            crawled_page: CrawledPage object containing HTML content

        Returns:
            Clean text content extracted from the page
        """
        try:
            clean_content = self.clean_html_content(crawled_page.html)
            # Update the processed content in the page object
            crawled_page.processed_content = clean_content
            return clean_content
        except Exception as e:
            logger.error(f"Error extracting content from page {crawled_page.url}: {str(e)}")
            raise TextProcessingException(f"Error extracting content from page {crawled_page.url}: {str(e)}")

    def remove_boilerplate_content(self, text: str) -> str:
        """
        Remove common boilerplate content from text, but preserve documentation content.

        Args:
            text: Raw text content

        Returns:
            Text content with boilerplate removed
        """
        # For documentation sites, we want to preserve more content
        # Only remove truly problematic patterns that aren't documentation
        lines = text.split('\n')
        filtered_lines = []

        for line in lines:
            line = line.strip()
            if line:
                # Skip lines that look like true boilerplate
                if not self._is_boilerplate_line(line):
                    filtered_lines.append(line)

        return '\n'.join(filtered_lines)

    def _is_boilerplate_line(self, line: str) -> bool:
        """
        Determine if a line is likely boilerplate content (more lenient for documentation).

        Args:
            line: Text line to check

        Returns:
            True if the line is likely boilerplate, False otherwise
        """
        # Convert to lowercase for comparison
        lower_line = line.lower()

        # True boilerplate patterns that should be removed
        boilerplate_patterns = [
            'copyright',
            'all rights reserved',
            'privacy policy',
            'terms of service',
            'contact us',
            'login',
            'register',
            'sign up',
            'sign in',
            'cookie policy',
            'powered by',
            'built with'
        ]

        # Check if the line contains any boilerplate patterns
        for pattern in boilerplate_patterns:
            if pattern in lower_line:
                return True

        # For documentation, be more lenient about short lines
        # since they might be headings, list items, or important concepts
        return False


def clean_html_content(html_content: str) -> str:
    """
    Convenience function to clean HTML content.

    Args:
        html_content: Raw HTML content

    Returns:
        Clean text content
    """
    cleaner = TextCleaner()
    return cleaner.clean_html_content(html_content)


def extract_content_from_page(crawled_page: CrawledPage) -> str:
    """
    Convenience function to extract content from a crawled page.

    Args:
        crawled_page: CrawledPage object containing HTML content

    Returns:
        Clean text content extracted from the page
    """
    cleaner = TextCleaner()
    return cleaner.extract_content_from_page(crawled_page)