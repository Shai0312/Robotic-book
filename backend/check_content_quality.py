#!/usr/bin/env python3
"""
Script to check the content quality in Qdrant after pipeline run.
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

print("Checking Qdrant Content Quality")
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
        # Get sample records to check content quality
        records = client.scroll(
            collection_name=collection_name,
            limit=10  # Get first 10 records
        )

        print(f"\nAnalyzing {len(records[0])} sample documents:")
        for i, record in enumerate(records[0]):
            payload = record.payload
            content = payload.get('content', '')
            source_url = payload.get('source_url', 'N/A')
            title = payload.get('title', 'N/A')

            print(f"\nDocument {i+1}:")
            print(f"  URL: {source_url}")
            print(f"  Title: {title}")
            print(f"  Content length: {len(content)} characters")
            print(f"  Content preview: {content[:200]}{'...' if len(content) > 200 else ''}")

            # Check if content looks substantial (not just a single line)
            lines = content.split('\n')
            non_empty_lines = [line for line in lines if line.strip()]
            print(f"  Non-empty lines: {len(non_empty_lines)}")

            if len(non_empty_lines) < 3:
                print(f"  ⚠️  WARNING: This document has very few lines ({len(non_empty_lines)})")

except Exception as e:
    print(f"Error connecting to Qdrant: {e}")