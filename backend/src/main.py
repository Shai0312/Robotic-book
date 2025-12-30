#!/usr/bin/env python3
"""
Main entry point for the RAG Knowledge Ingestion Pipeline.

This script orchestrates the complete process of crawling Docusaurus documentation,
extracting content, generating embeddings, and storing them in Qdrant.
"""
import argparse
import sys
import time
from typing import List

from src.crawler.docusaurus_crawler import DocusaurusCrawler
from src.text_processor.text_cleaner import TextCleaner
from src.text_processor.text_chunker import chunk_text
from src.embedding.cohere_embedder import CohereEmbedder
from src.storage.qdrant_storage import QdrantStorage
from src.models.document_chunk import DocumentChunk
from src.utils.logger import get_logger
from src.config.settings import settings


logger = get_logger(__name__)


def main_pipeline(urls: List[str], chunk_size: int = None, chunk_overlap: int = None) -> bool:
    """
    Execute the complete RAG knowledge ingestion pipeline.

    Args:
        urls: List of URLs to crawl and process
        chunk_size: Size of text chunks (defaults to setting)
        chunk_overlap: Overlap between chunks (defaults to setting)

    Returns:
        True if the pipeline completed successfully, False otherwise
    """
    logger.info("Starting RAG Knowledge Ingestion Pipeline")

    # Initialize components
    crawler = DocusaurusCrawler()
    cleaner = TextCleaner()
    embedder = CohereEmbedder()
    storage = QdrantStorage()

    all_document_chunks = []

    try:
        for url in urls:
            logger.info(f"Processing URL: {url}")

            # 1. Crawl the site
            logger.info("Step 1: Crawling website...")
            crawled_pages = crawler.crawl_site(url)
            logger.info(f"Crawled {len(crawled_pages)} pages")

            # 2. Clean and extract content from crawled pages
            logger.info("Step 2: Cleaning and extracting content...")
            for page in crawled_pages:
                clean_content = cleaner.extract_content_from_page(page)
                if clean_content.strip():  # Only process pages with content
                    # Chunk the content
                    chunks = chunk_text(
                        clean_content,
                        source_url=page.url,
                        title=page.title,
                        chunk_size=chunk_size or settings.CHUNK_SIZE,
                        chunk_overlap=chunk_overlap or settings.CHUNK_OVERLAP
                    )
                    all_document_chunks.extend(chunks)

            logger.info(f"Extracted and chunked content from {url}")

        logger.info(f"Total document chunks generated: {len(all_document_chunks)}")

        if not all_document_chunks:
            logger.warning("No content was extracted from the provided URLs")
            return False

        # 3. Generate embeddings
        logger.info("Step 3: Generating embeddings...")
        embedding_records = embedder.embed_document_chunks(all_document_chunks)
        logger.info(f"Generated embeddings for {len(embedding_records)} chunks")

        # 4. Store embeddings in Qdrant
        logger.info("Step 4: Storing embeddings in Qdrant...")
        success = storage.store_embeddings(embedding_records)
        if success:
            logger.info("Successfully stored embeddings in Qdrant")
        else:
            logger.error("Failed to store embeddings in Qdrant")
            return False

        logger.info("RAG Knowledge Ingestion Pipeline completed successfully")
        return True

    except Exception as e:
        logger.error(f"Error in main pipeline: {str(e)}")
        return False


def main():
    """Main function to parse arguments and run the pipeline."""
    parser = argparse.ArgumentParser(description="RAG Knowledge Ingestion Pipeline")
    parser.add_argument(
        "--urls",
        nargs="+",
        required=True,
        help="URL(s) to crawl and process"
    )
    parser.add_argument(
        "--chunk-size",
        type=int,
        default=settings.CHUNK_SIZE,
        help=f"Size of text chunks (default: {settings.CHUNK_SIZE})"
    )
    parser.add_argument(
        "--chunk-overlap",
        type=int,
        default=settings.CHUNK_OVERLAP,
        help=f"Overlap between chunks (default: {settings.CHUNK_OVERLAP})"
    )

    args = parser.parse_args()

    logger.info(f"Starting pipeline with URLs: {args.urls}")

    success = main_pipeline(
        urls=args.urls,
        chunk_size=args.chunk_size,
        chunk_overlap=args.chunk_overlap
    )

    if success:
        logger.info("Pipeline completed successfully")
        sys.exit(0)
    else:
        logger.error("Pipeline failed")
        sys.exit(1)


if __name__ == "__main__":
    main()