#!/usr/bin/env python3
"""
Demo script for the RAG Knowledge Ingestion Pipeline.

This script demonstrates how to use the pipeline with a sample URL.
Note: This is for demonstration purposes only. For actual use,
replace the URL with a real Docusaurus documentation site.
"""
import os
import sys
from pathlib import Path

# Add the src directory to the path so we can import modules
sys.path.insert(0, str(Path(__file__).parent / "src"))

from src.crawler import DocusaurusCrawler
from src.text_processor import TextCleaner, chunk_text
from src.embedding import CohereEmbedder
from src.storage import QdrantStorage
from src.utils.logger import get_logger


def demo_pipeline():
    """Demonstrate the RAG Knowledge Ingestion Pipeline."""
    logger = get_logger(__name__)
    logger.info("Starting RAG Knowledge Ingestion Pipeline Demo")

    # Check if required environment variables are set
    if not os.getenv("COHERE_API_KEY"):
        logger.error("COHERE_API_KEY environment variable is not set")
        return False

    if not os.getenv("QDRANT_URL"):
        logger.error("QDRANT_URL environment variable is not set")
        return False

    try:
        # Initialize components
        logger.info("Initializing pipeline components...")
        crawler = DocusaurusCrawler()
        cleaner = TextCleaner()
        embedder = CohereEmbedder()
        storage = QdrantStorage()

        # For demo purposes, we'll use a simple text instead of crawling
        # In a real scenario, you would crawl an actual documentation site
        sample_text = """
        This is a sample documentation page.
        It contains important information about a product or service.
        The RAG pipeline will crawl documentation sites like Docusaurus,
        extract the content, clean it, chunk it into smaller pieces,
        generate embeddings using Cohere models, and store them in Qdrant.

        Additional content to make the text long enough for chunking.
        More documentation content here to demonstrate the text processing
        capabilities of our RAG knowledge ingestion pipeline.
        """

        logger.info("Processing sample content...")

        # Create a mock crawled page
        from src.models.crawled_page import CrawledPage
        page = CrawledPage(
            url="https://example.com/docs/intro",
            title="Introduction",
            content=sample_text,
            html=f"<html><body>{sample_text}</body></html>",
            status="success"
        )

        # Clean the content
        clean_content = cleaner.extract_content_from_page(page)
        logger.info(f"Cleaned content length: {len(clean_content)} characters")

        # Chunk the content
        chunks = chunk_text(
            clean_content,
            source_url=page.url,
            title=page.title
        )
        logger.info(f"Created {len(chunks)} text chunks")

        # Generate embeddings
        embedding_records = embedder.embed_document_chunks(chunks)
        logger.info(f"Generated embeddings for {len(embedding_records)} chunks")

        # Store in Qdrant (this would work if Qdrant is configured)
        success = storage.store_embeddings(embedding_records)
        if success:
            logger.info("Successfully stored embeddings in Qdrant")
        else:
            logger.warning("Failed to store embeddings (may be due to Qdrant configuration)")

        logger.info("Demo completed successfully!")
        return True

    except Exception as e:
        logger.error(f"Error in demo: {str(e)}")
        return False


if __name__ == "__main__":
    success = demo_pipeline()
    if success:
        print("Demo completed successfully!")
        sys.exit(0)
    else:
        print("Demo failed!")
        sys.exit(1)