#!/usr/bin/env python3
"""
Test alternative URL patterns to access documentation content.
"""

import requests
from urllib.parse import urljoin

def check_url_accessibility(url, timeout=10):
    """Check if a URL is accessible."""
    try:
        response = requests.get(url, timeout=timeout, headers={'User-Agent': 'Mozilla/5.0 (compatible; RAG-Crawler/1.0)'})
        return response.status_code < 400
    except:
        return False

def main():
    base_url = "https://robotic-book-murex.vercel.app"

    print("Testing Alternative URL Patterns")
    print("=" * 50)

    # Test different URL patterns that might work
    test_urls = [
        # Standard Docusaurus patterns
        f"{base_url}/docs",
        f"{base_url}/docs/",
        f"{base_url}/en/",
        f"{base_url}/en/intro",
        f"{base_url}/en/docs",
        f"{base_url}/en/docs/intro",
        f"{base_url}/docs/en/",
        f"{base_url}/docs/en/intro",

        # API-style patterns
        f"{base_url}/api/docs",
        f"{base_url}/api/documentation",

        # Common documentation patterns
        f"{base_url}/documentation",
        f"{base_url}/guides",
        f"{base_url}/tutorials",
        f"{base_url}/manual",

        # Versioned patterns
        f"{base_url}/v1/docs",
        f"{base_url}/docs/v1",

        # Module-specific patterns from sitemap
        f"{base_url}/docs/Module-1-The-Robotic-Nervous-System",
        f"{base_url}/docs/Module-2-The-Digital-Twin",
        f"{base_url}/docs/Module-3-The-AI-Robot-Brain-NVIDIA-Isaac",
        f"{base_url}/docs/Module-4-Ch-1-Voice-to-Action",
        f"{base_url}/docs/ros2-concepts",
        f"{base_url}/docs/python-ros-integration",
        f"{base_url}/docs/urdf-humanoid-modeling",
        f"{base_url}/docs/digital-twin-gazebo-unity",

        # With and without trailing slashes
        f"{base_url}/docs/intro/",
        f"{base_url}/docs/ros2-concepts/",
        f"{base_url}/docs/python-ros-integration/",
        f"{base_url}/docs/urdf-humanoid-modeling/",
    ]

    accessible_urls = []

    print("Checking accessibility of alternative URL patterns:")
    for i, url in enumerate(test_urls):
        is_accessible = check_url_accessibility(url)
        status = "[OK]" if is_accessible else "[ERR]"
        print(f"  {status} {url}")

        if is_accessible:
            accessible_urls.append(url)

    print(f"\nFound {len(accessible_urls)} accessible URLs:")
    for url in accessible_urls:
        print(f"  - {url}")

    # Try with different headers that might work
    print(f"\nTrying with different headers...")
    headers_to_try = [
        {'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36'},
        {'User-Agent': 'Mozilla/5.0 (compatible; Googlebot/2.1)'},
        {'User-Agent': 'Mozilla/5.0', 'Accept': 'text/html,application/xhtml+xml,application/xml;q=0.9,*/*;q=0.8'},
    ]

    additional_accessible = []
    for url in [
        f"{base_url}/docs",
        f"{base_url}/docs/intro",
        f"{base_url}/documentation"
    ]:
        for header in headers_to_try:
            try:
                response = requests.get(url, headers=header, timeout=10)
                if response.status_code < 400 and url not in accessible_urls:
                    print(f"  [OK with headers] {url}")
                    additional_accessible.append(url)
                    break
            except:
                continue

    if additional_accessible:
        print(f"Additional accessible URLs with special headers: {additional_accessible}")
        accessible_urls.extend(additional_accessible)

    return accessible_urls

if __name__ == "__main__":
    accessible_urls = main()