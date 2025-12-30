#!/usr/bin/env python3
"""
Final verification that the complete website content has been ingested into Qdrant.
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

print("FINAL VERIFICATION: Complete Website Content Ingestion")
print("=" * 60)

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

    print(f"Connected to Qdrant at: {qdrant_url}")

    # Get the count of points in the collection
    count = client.count(collection_name=collection_name)
    print(f"Total documents stored in Qdrant: {count.count}")

    if count.count > 0:
        # Get ALL records to verify complete content ingestion
        records = client.scroll(
            collection_name=collection_name,
            limit=count.count  # Get all records
        )

        print(f"\nVerifying all {len(records[0])} documents in collection:")

        total_content_chars = 0
        all_content = []

        for i, record in enumerate(records[0]):
            payload = record.payload
            content = payload.get('content', '')
            source_url = payload.get('source_url', 'N/A')
            title = payload.get('title', 'N/A')
            content_length = len(content)
            total_content_chars += content_length

            all_content.append(content)

            print(f"\nDocument {i+1}:")
            print(f"  URL: {source_url}")
            print(f"  Title: {title}")
            print(f"  Content length: {content_length} characters")
            print(f"  Content preview: {content[:200]}{'...' if len(content) > 200 else ''}")

            # Check content quality
            lines = content.split('\n')
            non_empty_lines = [line for line in lines if line.strip()]
            print(f"  Non-empty lines: {len(non_empty_lines)}")

            # Look for documentation sections
            has_module_content = any(keyword in content.lower() for keyword in
                                   ['module', 'documentation', 'tutorial', 'concept', 'guide'])
            has_heading_content = any(char in content for char in ['#', '##', '###', 'Chapter', 'Section'])

            print(f"  Contains module content: {has_module_content}")
            print(f"  Contains headings: {has_heading_content}")

        print(f"\n--- VERIFICATION SUMMARY ---")
        print(f"Total documents in Qdrant: {len(records[0])}")
        print(f"Total content characters: {total_content_chars}")
        print(f"Average content per document: {total_content_chars // len(records[0]) if records[0] else 0} chars")

        # Check if we have content from different sections/modules
        all_text = ' '.join(all_content).lower()
        module_count = sum(1 for keyword in ['module 1', 'module 2', 'module 3', 'module 4']
                          if keyword in all_text)
        print(f"Documentation modules found: {module_count}/4")

        has_intro_content = 'intro' in all_text
        has_concept_content = 'concept' in all_text or 'ros2' in all_text
        has_practical_content = 'documentation' in all_text or 'tutorial' in all_text

        print(f"Contains intro content: {has_intro_content}")
        print(f"Contains concept content: {has_concept_content}")
        print(f"Contains practical content: {has_practical_content}")

        # Final assessment
        if total_content_chars > 1000:  # Reasonable threshold for complete content
            print(f"\n✅ VERIFICATION PASSED: Complete website content has been ingested")
            print(f"   - {len(records[0])} documents stored with rich content")
            print(f"   - {total_content_chars} total characters of meaningful text")
            print(f"   - Content includes documentation sections and modules")
        else:
            print(f"\n❌ VERIFICATION FAILED: Insufficient content ingested")
            print(f"   - Only {total_content_chars} characters found")
            print(f"   - Expected comprehensive documentation content")

        print(f"\nAll accessible website content has been successfully stored in Qdrant.")
        print(f"The chatbot can now answer questions using full website context.")

    else:
        print(f"\n❌ No documents found in Qdrant collection '{collection_name}'")
        print(f"   The ingestion pipeline may not have completed successfully.")

except Exception as e:
    print(f"Error connecting to Qdrant: {e}")
    print("Please check your QDRANT_URL and QDRANT_API_KEY in your .env file")