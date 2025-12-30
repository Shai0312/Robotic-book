#!/usr/bin/env python3
"""
Debug script to examine the actual content stored in Qdrant.
"""

import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient

# Load environment variables from .env file
load_dotenv()

# Load your Qdrant configuration from environment
qdrant_url = os.getenv("QDRANT_URL")
qdrant_api_key = os.getenv("QDRANT_API_KEY")
collection_name = os.getenv("QDRANT_COLLECTION_NAME", "documents")

print("Qdrant Content Debug")
print("=" * 50)

try:
    # Initialize Qdrant client with proper configuration for cloud
    if qdrant_api_key:
        # For Qdrant Cloud
        client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key,
            prefer_grpc=False
        )
    else:
        # For local instance
        client = QdrantClient(url=qdrant_url)

    # Get the count of points in the collection
    count = client.count(collection_name=collection_name)
    print(f"Total documents in Qdrant: {count.count}")

    if count.count > 0:
        # Get ALL records to check the complete content
        records = client.scroll(
            collection_name=collection_name,
            limit=count.count  # Get all records
        )

        print(f"\nExamining all {len(records[0])} documents in collection:")

        total_content_chars = 0

        for i, record in enumerate(records[0]):
            payload = record.payload
            content = payload.get('content', '')
            source_url = payload.get('source_url', 'N/A')
            title = payload.get('title', 'N/A')
            content_length = len(content)
            total_content_chars += content_length

            print(f"\n--- Document {i+1} ---")
            print(f"ID: {record.id}")
            print(f"URL: {source_url}")
            print(f"Title: {title}")
            print(f"Content length: {content_length} characters")
            print(f"Content preview (first 200 chars): {content[:200]}{'...' if len(content) > 200 else ''}")

            # Check if content looks substantial
            lines = content.split('\n')
            non_empty_lines = [line for line in lines if line.strip()]
            print(f"Non-empty lines: {len(non_empty_lines)}")

            if content_length < 100:
                print(f"⚠️  WARNING: This document has very little content ({content_length} chars)")
            if len(non_empty_lines) < 3:
                print(f"⚠️  WARNING: This document has very few lines ({len(non_empty_lines)})")

        print(f"\n--- SUMMARY ---")
        print(f"Total documents: {len(records[0])}")
        print(f"Total content characters: {total_content_chars}")
        print(f"Average content per document: {total_content_chars // len(records[0]) if records[0] else 0} chars")

        # Fetch the original page to compare
        print(f"\n--- COMPARISON WITH ORIGINAL PAGE ---")
        import requests
        original_response = requests.get("https://robotic-book-murex.vercel.app/")
        original_length = len(original_response.text)
        print(f"Original HTML length: {original_length} characters")

        # Estimate how much content we should have extracted
        # Assuming about 10-15% of HTML is actual content
        estimated_content_size = original_length * 0.1
        print(f"Estimated content size: ~{estimated_content_size} characters")
        print(f"Actual extracted content: {total_content_chars} characters")
        print(f"Extraction ratio: {(total_content_chars / original_length * 100):.2f}%")

except Exception as e:
    print(f"Error connecting to Qdrant: {e}")
    import traceback
    traceback.print_exc()