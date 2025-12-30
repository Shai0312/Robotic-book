"""
Helper functions for the RAG knowledge ingestion pipeline.
"""
import re
from urllib.parse import urljoin, urlparse
from typing import List, Optional
import requests
from src.utils.exceptions import URLValidationException


def is_valid_url(url: str) -> bool:
    """
    Validate if a string is a properly formatted URL.

    Args:
        url: URL string to validate

    Returns:
        True if the URL is valid, False otherwise
    """
    try:
        result = urlparse(url)
        return all([result.scheme, result.netloc])
    except Exception:
        return False


def normalize_url(url: str) -> str:
    """
    Normalize a URL by ensuring it has the proper scheme and removing trailing slashes.

    Args:
        url: URL string to normalize

    Returns:
        Normalized URL string
    """
    if not url.startswith(('http://', 'https://')):
        url = 'https://' + url

    # Remove trailing slash if present (but keep the scheme)
    if url.endswith('/'):
        url = url.rstrip('/')

    return url


def is_same_domain(base_url: str, test_url: str) -> bool:
    """
    Check if two URLs belong to the same domain.

    Args:
        base_url: The base URL to compare against
        test_url: The URL to test

    Returns:
        True if both URLs are from the same domain, False otherwise
    """
    base_domain = urlparse(base_url).netloc
    test_domain = urlparse(test_url).netloc
    return base_domain == test_domain


def extract_links_from_html(html_content: str, base_url: str) -> List[str]:
    """
    Extract all links from HTML content that belong to the same domain as the base URL.

    Args:
        html_content: HTML content to extract links from
        base_url: Base URL to resolve relative links against

    Returns:
        List of absolute URLs found in the HTML content
    """
    import re
    from urllib.parse import urljoin, urlparse

    # Regular expression to find href attributes in anchor tags
    link_pattern = r'<a[^>]*href\s*=\s*["\']([^"\']*)["\'][^>]*>'
    matches = re.findall(link_pattern, html_content, re.IGNORECASE)

    links = []
    for match in matches:
        # Skip anchor-only links (like #section) that point to the same page
        if match.strip().startswith('#'):
            continue

        # Skip empty links
        if not match.strip():
            continue

        # Resolve relative URLs to absolute URLs
        absolute_url = urljoin(base_url, match.strip())
        if is_valid_url(absolute_url) and is_same_domain(base_url, absolute_url):
            # Avoid adding the same URL with different anchors (they're the same content)
            # Remove fragment part (#...) for comparison to avoid duplicates
            url_without_fragment = absolute_url.split('#')[0]
            if url_without_fragment not in [link.split('#')[0] for link in links]:
                links.append(absolute_url)

    # Remove duplicates while preserving order (already handled above, but keeping as safety)
    unique_links = []
    for link in links:
        link_without_fragment = link.split('#')[0]
        if link_without_fragment not in [ul.split('#')[0] for ul in unique_links]:
            unique_links.append(link)

    return unique_links


def validate_and_format_url(url: str) -> str:
    """
    Validate and format a URL, raising an exception if it's invalid.

    Args:
        url: URL string to validate and format

    Returns:
        Validated and formatted URL

    Raises:
        URLValidationException: If the URL is invalid
    """
    if not url:
        raise URLValidationException("URL cannot be empty")

    normalized_url = normalize_url(url)

    if not is_valid_url(normalized_url):
        raise URLValidationException(f"Invalid URL format: {url}")

    return normalized_url


def clean_text_content(text: str) -> str:
    """
    Clean extracted text content by removing extra whitespace and normalizing.

    Args:
        text: Raw text content to clean

    Returns:
        Cleaned text content
    """
    if not text:
        return ""

    # Replace multiple whitespace characters with a single space
    cleaned = re.sub(r'\s+', ' ', text)

    # Strip leading and trailing whitespace
    cleaned = cleaned.strip()

    return cleaned


def check_url_accessibility(url: str, timeout: int = 10) -> bool:
    """
    Check if a URL is accessible by making a HEAD request.

    Args:
        url: URL to check
        timeout: Request timeout in seconds

    Returns:
        True if the URL is accessible (status code < 400), False otherwise
    """
    try:
        response = requests.head(url, timeout=timeout, allow_redirects=True)
        return response.status_code < 400
    except requests.RequestException:
        return False