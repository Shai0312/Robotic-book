#!/usr/bin/env python3
"""
Debug script to check exactly how the sitemap is being parsed and processed.
"""

import requests
import re
from urllib.parse import urljoin, urlparse

def is_same_domain(base_url, test_url):
    """Check if two URLs belong to the same domain."""
    base_domain = urlparse(base_url).netloc
    test_domain = urlparse(test_url).netloc
    return base_domain == test_domain

def check_url_accessibility(url, timeout=10):
    """Check if a URL is accessible using GET request."""
    try:
        response = requests.get(url, timeout=timeout, headers={'User-Agent': 'Mozilla/5.0 (compatible; RAG-Crawler/1.0)'})
        return response.status_code < 400
    except:
        return False

def main():
    base_url = "https://robotic-book-murex.vercel.app"

    print("Debugging Sitemap Processing")
    print("=" * 50)

    # Fetch the sitemap
    sitemap_url = urljoin(base_url, 'sitemap.xml')
    print(f"Fetching sitemap: {sitemap_url}")

    try:
        response = requests.get(sitemap_url)
        sitemap_content = response.text
        print(f"Sitemap fetched successfully, length: {len(sitemap_content)}")
    except Exception as e:
        print(f"Could not fetch sitemap: {e}")
        return []

    # Extract URLs from sitemap XML using the same pattern as the crawler
    url_pattern = r'<loc>(https?://[^<]+)</loc>'
    found_urls = re.findall(url_pattern, sitemap_content, re.IGNORECASE)

    print(f"Found {len(found_urls)} URLs in sitemap")

    # Filter URLs to only include those from the same domain
    valid_urls = []
    for i, url in enumerate(found_urls):
        if is_same_domain(base_url, url):
            # Check if the URL is accessible before adding to the list
            print(f"Checking URL {i+1}: {url}")
            if check_url_accessibility(url):
                print(f"  [OK] Accessible")
                valid_urls.append(url)
            else:
                print(f"  [ERR] Not accessible (404 or error)")
        else:
            print(f"  - Different domain, skipping: {url}")

    print(f"\nFinal result: {len(valid_urls)} accessible URLs from sitemap")

    # Show a sample of the accessible URLs
    if valid_urls:
        print(f"\nAccessible URLs (first 10):")
        for i, url in enumerate(valid_urls[:10]):
            print(f"  {i+1}. {url}")
        if len(valid_urls) > 10:
            print(f"  ... and {len(valid_urls) - 10} more")
    else:
        print("\nNo accessible URLs found!")

        # Let's also check some specific documentation URLs manually
        print(f"\nTesting specific documentation URLs manually:")
        test_urls = [
            "https://robotic-book-murex.vercel.app/docs",
            "https://robotic-book-murex.vercel.app/docs/intro",
            "https://robotic-book-murex.vercel.app/docs/ros2-concepts",
            "https://robotic-book-murex.vercel.app/docs/python-ros-integration"
        ]

        for test_url in test_urls:
            is_accessible = check_url_accessibility(test_url)
            status = "[OK]" if is_accessible else "[ERR]"
            print(f"  {status} {test_url}")

    return valid_urls

if __name__ == "__main__":
    accessible_urls = main()