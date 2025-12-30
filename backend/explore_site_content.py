#!/usr/bin/env python3
"""
Explore alternative ways to find content on the website beyond sitemap.
"""

import requests
from bs4 import BeautifulSoup
from urllib.parse import urljoin, urlparse
import re

def extract_links_from_html(html_content, base_url):
    """Extract all links from HTML content."""
    soup = BeautifulSoup(html_content, 'html.parser')

    links = []
    for link in soup.find_all('a', href=True):
        href = link['href']
        # Resolve relative URLs to absolute URLs
        absolute_url = urljoin(base_url, href)
        links.append(absolute_url)

    # Remove duplicates while preserving order
    unique_links = []
    for link in links:
        if link not in unique_links:
            unique_links.append(link)

    return unique_links

def check_url_accessibility(url, timeout=10):
    """Check if a URL is accessible."""
    try:
        response = requests.get(url, timeout=timeout, headers={'User-Agent': 'Mozilla/5.0 (compatible; RAG-Crawler/1.0)'})
        return response.status_code < 400
    except:
        return False

def main():
    base_url = "https://robotic-book-murex.vercel.app"

    print("Exploring Alternative Content Sources")
    print("=" * 50)

    # First, get links from the main page
    print("Fetching main page...")
    try:
        response = requests.get(base_url)
        main_page_content = response.text
    except Exception as e:
        print(f"Could not fetch main page: {e}")
        return

    print("Extracting links from main page...")
    main_page_links = extract_links_from_html(main_page_content, base_url)

    print(f"Found {len(main_page_links)} links on main page")

    # Filter for internal links only
    internal_links = []
    for link in main_page_links:
        if urlparse(link).netloc == urlparse(base_url).netloc:
            internal_links.append(link)

    print(f"Found {len(internal_links)} internal links")

    # Check accessibility of internal links
    accessible_links = []
    print("\nChecking accessibility of internal links:")
    for i, link in enumerate(internal_links):
        is_accessible = check_url_accessibility(link)
        status = "[OK]" if is_accessible else "[ERR]"
        print(f"  {status} {link}")

        if is_accessible:
            accessible_links.append(link)

    print(f"\nTotal accessible internal links: {len(accessible_links)}")

    # Also check for other potential documentation URLs that might not be in sitemap
    print("\nChecking common documentation patterns:")
    common_docs_paths = [
        "/docs",
        "/documentation",
        "/tutorials",
        "/guides",
        "/api",
        "/reference",
        "/manual",
        "/help",
        "/wiki"
    ]

    for path in common_docs_paths:
        full_url = urljoin(base_url, path)
        is_accessible = check_url_accessibility(full_url)
        status = "[OK]" if is_accessible else "[ERR]"
        print(f"  {status} {full_url}")

        if is_accessible and full_url not in accessible_links:
            accessible_links.append(full_url)

    print(f"\nFinal count of accessible URLs: {len(accessible_links)}")

    if accessible_links:
        print("\nAccessible URLs:")
        for url in accessible_links:
            print(f"  - {url}")
    else:
        print("\nNo additional accessible URLs found.")
        print("The site may have a deployment configuration issue where documentation pages return 404 errors.")

    return accessible_links

if __name__ == "__main__":
    accessible_urls = main()