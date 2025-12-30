import os
from qdrant_client import QdrantClient
from qdrant_client.http import models

# Load your Qdrant configuration from environment
qdrant_url = os.getenv("QDRANT_URL")
qdrant_api_key = os.getenv("QDRANT_API_KEY")
collection_name = os.getenv("QDRANT_COLLECTION_NAME", "documents")

print(f"QDRANT_URL: {qdrant_url}")
print(f"QDRANT_COLLECTION_NAME: {collection_name}")

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

    print("Connected to Qdrant successfully!")

    # List all collections
    collections = client.get_collections()
    print(f"Available collections: {[col.name for col in collections.collections]}")

    # Check if our collection exists
    if collection_name in [col.name for col in collections.collections]:
        print(f"Collection '{collection_name}' exists!")

        # Get the count of points in the collection
        count = client.count(collection_name=collection_name)
        print(f"Number of points in '{collection_name}': {count.count}")

        # Try to get first few records to verify
        if count.count > 0:
            records = client.scroll(
                collection_name=collection_name,
                limit=5  # Get first 5 records
            )
            print(f"Sample records: {len(records[0])} found")
            for i, record in enumerate(records[0][:2]):  # Print first 2 records
                print(f"  Record {i+1}: ID={record.id}, Payload keys: {list(record.payload.keys()) if record.payload else 'None'}")
        else:
            print(f"Collection '{collection_name}' is empty.")
    else:
        print(f"Collection '{collection_name}' does not exist.")

except Exception as e:
    print(f"Error connecting to Qdrant: {e}")
    print("Please check your QDRANT_URL and QDRANT_API_KEY in your .env file")
    print("For Qdrant Cloud, URL should look like: https://your-cluster-id.gcp.cloud.qdrant.io:6333")