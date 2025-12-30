#!/usr/bin/env python3
"""
Verify that all processed content is being stored in Qdrant.
"""

import os
import requests
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from src.text_processor.text_cleaner import TextCleaner
from src.text_processor.text_chunker import TextChunker
from src.embedding.cohere_embedder import CohereEmbedder

# Load environment variables from .env file
load_dotenv()

# Load your Qdrant configuration from environment
qdrant_url = os.getenv("QDRANT_URL")
qdrant_api_key = os.getenv("QDRANT_API_KEY")
collection_name = os.getenv("QDRANT_COLLECTION_NAME", "documents")

print("Storage Verification")
print("=" * 50)

# Fetch the main page content
url = "https://robotic-book-murex.vercel.app/"
response = requests.get(url)
html_content = response.text

# Process the content exactly as the pipeline does
cleaner = TextCleaner()
cleaned_content = cleaner.clean_html_content(html_content)

chunker = TextChunker(chunk_size=512, chunk_overlap=50)
chunks = chunker.chunk_text(cleaned_content, source_url=url, title="Hello from Physical AI Humanoid Robotics | Physical AI Humanoid Robotics")

print(f"Content processing results:")
print(f"- Original HTML: {len(html_content)} characters")
print(f"- Cleaned content: {len(cleaned_content)} characters")
print(f"- Number of chunks: {len(chunks)}")
print(f"- Total content in chunks: {sum(len(chunk.content) for chunk in chunks)} characters (with overlap)")

# Check what's in Qdrant
if qdrant_api_key:
    client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key, prefer_grpc=False)
else:
    client = QdrantClient(url=qdrant_url)

count = client.count(collection_name=collection_name)
print(f"- Documents in Qdrant: {count.count}")

if count.count > 0:
    records = client.scroll(collection_name=collection_name, limit=count.count)

    qdrant_total_content = sum(len(record.payload.get('content', '')) for record in records[0])
    print(f"- Total content in Qdrant: {qdrant_total_content} characters")

    print(f"\nChunk details:")
    for i, chunk in enumerate(chunks):
        print(f"  Chunk {i+1}: {len(chunk.content)} chars")

    print(f"\nQdrant records:")
    for i, record in enumerate(records[0]):
        content_length = len(record.payload.get('content', ''))
        print(f"  Record {i+1}: {content_length} chars")

    # Compare content
    if len(chunks) == count.count:
        print(f"\n✅ Chunk count matches Qdrant records ({len(chunks)} == {count.count})")
    else:
        print(f"\n❌ Chunk count mismatch ({len(chunks)} != {count.count})")

    # Check if content lengths match approximately (considering overlap)
    if abs(sum(len(chunk.content) for chunk in chunks) - qdrant_total_content) < 50:  # Allow small differences
        print(f"✅ Content length matches approximately (diff: {abs(sum(len(chunk.content) for chunk in chunks) - qdrant_total_content)})")
    else:
        print(f"❌ Content length mismatch (pipeline: {sum(len(chunk.content) for chunk in chunks)}, qdrant: {qdrant_total_content})")

        print(f"\nDetailed comparison:")
        for i, chunk in enumerate(chunks):
            print(f"  Pipeline chunk {i+1}: {len(chunk.content)} chars")

        for i, record in enumerate(records[0]):
            content_length = len(record.payload.get('content', ''))
            print(f"  Qdrant record {i+1}: {content_length} chars")

print(f"\nStorage verification complete.")