#!/usr/bin/env python3
"""
Test script to check which documentation pages are accessible on your site.
This will help identify which URLs can be successfully crawled.
"""

import requests
from urllib.parse import urljoin
import time

def check_url_accessibility(url, timeout=10):
    """Check if a URL is accessible (returns 200 status)."""
    try:
        response = requests.head(url, timeout=timeout, allow_redirects=True)
        return response.status_code < 400
    except:
        return False

def main():
    base_url = "https://robotic-book-murex.vercel.app"

    # Common Docusaurus documentation paths that should work
    doc_paths = [
        "/",
        "/docs",
        "/docs/intro",
        "/docs/ros2-concepts",
        "/docs/ros2-concepts/introduction",
        "/docs/ros2-concepts/nodes",
        "/docs/ros2-concepts/topics-messages",
        "/docs/ros2-concepts/services",
        "/docs/ros2-concepts/learning-objectives",
        "/docs/ros2-concepts/data-flow-humanoid",
        "/docs/ros2-concepts/intro",
        "/docs/python-ros-integration",
        "/docs/python-ros-integration/intro-rclpy",
        "/docs/python-ros-integration/creating-nodes",
        "/docs/python-ros-integration/publish-subscribe",
        "/docs/python-ros-integration/using-services",
        "/docs/python-ros-integration/agent-controller-actuator",
        "/docs/urdf-humanoid-modeling",
        "/docs/urdf-humanoid-modeling/intro-urdf",
        "/docs/urdf-humanoid-modeling/links-joints",
        "/docs/urdf-humanoid-modeling/frames-transformations",
        "/docs/urdf-humanoid-modeling/xml-examples",
        "/docs/urdf-humanoid-modeling/practical-examples",
        "/docs/digital-twin-gazebo-unity",
        "/docs/digital-twin-gazebo-unity/chapter-1-gazebo-simulations",
        "/docs/digital-twin-gazebo-unity/chapter-1-gazebo-simulations/setup",
        "/docs/digital-twin-gazebo-unity/chapter-1-gazebo-simulations/environments",
        "/docs/digital-twin-gazebo-unity/chapter-1-gazebo-simulations/physics",
        "/docs/digital-twin-gazebo-unity/chapter-1-gazebo-simulations/sensors",
        "/docs/digital-twin-gazebo-unity/chapter-1-gazebo-simulations/practical-examples",
        "/docs/digital-twin-gazebo-unity/chapter-2-unity-digital-twins",
        "/docs/digital-twin-gazebo-unity/chapter-2-unity-digital-twins/unity-setup",
        "/docs/digital-twin-gazebo-unity/chapter-2-unity-digital-twins/digital-twin-modeling",
        "/docs/digital-twin-gazebo-unity/chapter-2-unity-digital-twins/hri-concepts",
        "/docs/digital-twin-gazebo-unity/chapter-2-unity-digital-twins/interaction-design",
        "/docs/digital-twin-gazebo-unity/chapter-2-unity-digital-twins/visualization-examples",
        "/docs/Module-1-The-Robotic-Nervous-System",
        "/docs/Module-2-The-Digital-Twin",
        "/docs/Module-3-The-AI-Robot-Brain-NVIDIA-Isaac",
        "/docs/Module-4-Ch-1-Voice-to-Action",
        "/docs/Module-4-Ch-2-Cognitive-Planning-with-LLMs",
        "/docs/Module-4-Ch-3-Capstone-Autonomous-Humanoid",
        "/docs/glossary",
        "/docs/contributing",
        "/docs/quickstart",
        "/markdown-page"
    ]

    print("Testing accessibility of documentation pages...")
    print("=" * 60)

    accessible_urls = []

    for path in doc_paths:
        full_url = urljoin(base_url, path)
        is_accessible = check_url_accessibility(full_url)

        status = "[OK]" if is_accessible else "[ERR]"
        print(f"{status} {full_url}")
        print(f"  Status: {'Accessible' if is_accessible else 'Not accessible (404 or error)'}")

        if is_accessible:
            accessible_urls.append(full_url)

        # Be respectful to the server
        time.sleep(0.1)

    print("=" * 60)
    print(f"Found {len(accessible_urls)} accessible documentation pages")

    if accessible_urls:
        print("\nAccessible URLs:")
        for url in accessible_urls:
            print(f"  - {url}")

    return accessible_urls

if __name__ == "__main__":
    accessible_urls = main()