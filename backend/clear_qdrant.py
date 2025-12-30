#!/usr/bin/env python3
"""
Script to clear the Qdrant collection and verify it's empty.
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

print("Clearing Qdrant Collection")
print("=" * 30)

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

    # Get the count before clearing
    count_before = client.count(collection_name=collection_name)
    print(f"Documents before clearing: {count_before.count}")

    if count_before.count > 0:
        # Delete all points in the collection
        client.delete_collection(collection_name)
        print(f"Deleted collection: {collection_name}")

        # Recreate the collection
        from qdrant_client.http import models
        client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(
                size=1024,  # Default size for Cohere embeddings
                distance=models.Distance.COSINE
            )
        )
        print(f"Recreated collection: {collection_name}")

    # Check count after clearing
    count_after = client.count(collection_name=collection_name)
    print(f"Documents after clearing: {count_after.count}")

    print("Collection is now ready for new data.")

except Exception as e:
    print(f"Error clearing Qdrant collection: {e}")