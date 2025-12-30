#!/usr/bin/env python3
"""
Extract and analyze the full content from the main accessible page.
"""

import requests
from bs4 import BeautifulSoup
from src.text_processor.text_cleaner import TextCleaner
from src.text_processor.text_chunker import TextChunker

# Fetch the main page
url = "https://robotic-book-murex.vercel.app/"
response = requests.get(url)
html_content = response.text

print("Main Page Content Analysis")
print("=" * 50)
print(f"HTML length: {len(html_content)} characters")

# Parse with BeautifulSoup to look for content structure
soup = BeautifulSoup(html_content, 'html.parser')

# Look for different content sections
headings = soup.find_all(['h1', 'h2', 'h3', 'h4', 'h5', 'h6'])
print(f"\nHeadings found: {len(headings)}")
for i, heading in enumerate(headings):
    print(f"  {i+1}. {heading.get_text().strip()}")

# Look for content sections
sections = soup.find_all(['section', 'article', 'main', 'div'], class_=lambda x: x and any(keyword in x.lower() for keyword in ['content', 'main', 'doc', 'section', 'container']))
print(f"\nPotential content sections found: {len(sections)}")

# Look for article or main content areas
main_content = soup.find('main') or soup.find('article') or soup.find('div', class_=lambda x: x and 'main' in x.lower()) or soup

# Apply the improved cleaner
cleaner = TextCleaner()
cleaned_content = cleaner.clean_html_content(html_content)

print(f"\nCleaned content length: {len(cleaned_content)} characters")
print(f"Cleaned content preview:")
print(cleaned_content[:500] + ("..." if len(cleaned_content) > 500 else ""))

# Check how it gets chunked
chunker = TextChunker(chunk_size=512, chunk_overlap=50)
chunks = chunker.chunk_text(cleaned_content, source_url=url, title="Main Page")

print(f"\nAfter chunking with size 512, overlap 50:")
print(f"Number of chunks: {len(chunks)}")
print(f"Total content across chunks: {sum(len(chunk.content) for chunk in chunks)} characters")

for i, chunk in enumerate(chunks):
    print(f"\nChunk {i+1} ({len(chunk.content)} chars):")
    print(f"  Preview: {chunk.content[:100]}{'...' if len(chunk.content) > 100 else ''}")

# Also look for content that might be organized by semantic elements
content_blocks = []
for tag in ['section', 'article', 'div']:
    elements = soup.find_all(tag, class_=lambda x: x and any(keyword in x.lower() for keyword in ['hero', 'features', 'docs', 'content', 'section']))
    content_blocks.extend(elements)

print(f"\nFound {len(content_blocks)} potential content blocks by class name")

# Look for content in navigation or sidebar that might be documentation structure
nav_links = soup.find_all('a', href=True)
doc_links = [link for link in nav_links if '/docs/' in link['href']]
print(f"\nFound {len(doc_links)} documentation links in navigation:")
for i, link in enumerate(doc_links[:10]):  # Show first 10
    print(f"  {i+1}. {link['href']} - {link.get_text().strip()}")

if len(doc_links) > 10:
    print(f"  ... and {len(doc_links) - 10} more")