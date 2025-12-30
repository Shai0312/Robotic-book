#!/usr/bin/env python3
"""
Test sitemap URLs using GET requests to see if they're accessible.
"""

import requests
from urllib.parse import urljoin
import time
import re

def check_url_with_get(url, timeout=10):
    """Check if a URL is accessible using GET request."""
    try:
        response = requests.get(url, timeout=timeout, headers={'User-Agent': 'Mozilla/5.0 (compatible; RAG-Crawler/1.0)'})
        return response.status_code < 400
    except:
        return False

def main():
    base_url = "https://robotic-book-murex.vercel.app"

    # Fetch the sitemap
    sitemap_url = urljoin(base_url, 'sitemap.xml')
    print(f"Fetching sitemap: {sitemap_url}")

    try:
        response = requests.get(sitemap_url)
        sitemap_content = response.text
    except Exception as e:
        print(f"Could not fetch sitemap: {e}")
        return []

    # Extract URLs from sitemap XML
    url_pattern = r'<loc>(https?://[^<]+)</loc>'
    found_urls = re.findall(url_pattern, sitemap_content, re.IGNORECASE)

    print(f"Found {len(found_urls)} URLs in sitemap")
    print("=" * 60)

    accessible_urls = []

    for i, full_url in enumerate(found_urls):
        is_accessible = check_url_with_get(full_url)

        status = "[OK]" if is_accessible else "[ERR]"
        print(f"{status} {full_url}")

        if is_accessible:
            accessible_urls.append(full_url)

        # Be respectful to the server
        time.sleep(0.1)

        # Show progress
        if (i + 1) % 10 == 0:
            print(f"Processed {i + 1}/{len(found_urls)} URLs...")

    print("=" * 60)
    print(f"Found {len(accessible_urls)} accessible URLs out of {len(found_urls)} total")

    if accessible_urls:
        print("\nAccessible URLs:")
        for url in accessible_urls:
            print(f"  - {url}")
    else:
        print("\nNo accessible URLs found!")

    return accessible_urls

if __name__ == "__main__":
    accessible_urls = main()