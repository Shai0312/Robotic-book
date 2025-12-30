#!/usr/bin/env python3
"""
Test to see the full content extracted from the page before chunking.
"""

import requests
from src.text_processor.text_cleaner import TextCleaner
from src.text_processor.text_chunker import TextChunker

# Fetch the main page
url = "https://robotic-book-murex.vercel.app/"
response = requests.get(url)
html_content = response.text

print("Original HTML length:", len(html_content))
print("\n" + "="*50 + "\n")

# Apply the improved cleaner
cleaner = TextCleaner()
cleaned_content = cleaner.clean_html_content(html_content)

print("Cleaned content length:", len(cleaned_content))
print("\nFull cleaned content:")
print(cleaned_content)
print(f"\nTotal lines in cleaned content: {len(cleaned_content.split(chr(10)))}")

# Now see how it gets chunked
chunker = TextChunker(chunk_size=512, chunk_overlap=50)
chunks = chunker.chunk_text(cleaned_content, source_url=url, title="Test Page")

print(f"\nAfter chunking: {len(chunks)} chunks")
for i, chunk in enumerate(chunks):
    print(f"\nChunk {i+1} length: {len(chunk.content)} characters")
    print(f"Chunk {i+1} preview: {chunk.content[:200]}{'...' if len(chunk.content) > 200 else ''}")

print(f"\nTotal content across all chunks: {sum(len(chunk.content) for chunk in chunks)} characters")