from typing import List
from src.utils.logger import get_logger
from src.config.settings import settings
from src.models.document_chunk import DocumentChunk
import uuid

logger = get_logger(__name__)


class TextChunker:
    """
    Chunks text content into smaller segments for embedding.
    """

    def __init__(self, chunk_size: int = None, chunk_overlap: int = None):
        """
        Initialize the TextChunker.

        Args:
            chunk_size: Size of each text chunk (default from settings)
            chunk_overlap: Overlap between chunks (default from settings)
        """
        self.chunk_size = chunk_size or settings.CHUNK_SIZE
        self.chunk_overlap = chunk_overlap or settings.CHUNK_OVERLAP

    def chunk_text(self, text: str, source_url: str = "", title: str = "") -> List[DocumentChunk]:
        """
        Split text into chunks of specified size with overlap.

        Args:
            text: Text to chunk
            source_url: URL of the source document
            title: Title of the source document

        Returns:
            List of DocumentChunk objects
        """
        if not text:
            return []

        chunks = []
        start = 0
        text_length = len(text)

        while start < text_length:
            # Determine the end position for this chunk
            end = start + self.chunk_size

            # If this is not the last chunk, try to break at a sentence or word boundary
            if end < text_length:
                # Look for a good breaking point near the end
                chunk_text = text[start:end]
                break_found = False

                # Try to break at sentence endings
                for separator in ['.\n', '. ', '! ', '? ', '; ', ': ']:
                    last_sep = chunk_text.rfind(separator)
                    if last_sep != -1:
                        end = start + last_sep + len(separator)
                        break_found = True
                        break

                # If no sentence boundary found, try word boundaries
                if not break_found:
                    for separator in ['\n', ' ', '-', '\t']:
                        last_sep = chunk_text.rfind(separator)
                        if last_sep != -1:
                            end = start + last_sep + len(separator)
                            break_found = True
                            break

            # Extract the chunk text
            chunk_text = text[start:end]

            # Create a DocumentChunk object
            chunk = DocumentChunk(
                id=str(uuid.uuid4()),
                content=chunk_text,
                source_url=source_url,
                title=title,
                metadata={
                    "start_pos": start,
                    "end_pos": end,
                    "chunk_index": len(chunks)
                }
            )

            chunks.append(chunk)

            # Move start position, considering overlap
            if end >= text_length:
                # This was the last chunk
                break
            else:
                # Move start position with overlap
                start = end - self.chunk_overlap

        logger.info(f"Text chunked into {len(chunks)} chunks")
        return chunks

    def chunk_multiple_texts(self, texts: List[str], source_urls: List[str] = None, titles: List[str] = None) -> List[DocumentChunk]:
        """
        Chunk multiple texts.

        Args:
            texts: List of texts to chunk
            source_urls: List of source URLs (optional)
            titles: List of titles (optional)

        Returns:
            List of DocumentChunk objects
        """
        all_chunks = []
        for i, text in enumerate(texts):
            source_url = source_urls[i] if source_urls and i < len(source_urls) else ""
            title = titles[i] if titles and i < len(titles) else f"Document {i}"
            chunks = self.chunk_text(text, source_url, title)
            all_chunks.extend(chunks)

        return all_chunks


def chunk_text(text: str, source_url: str = "", title: str = "", chunk_size: int = None, chunk_overlap: int = None) -> List[DocumentChunk]:
    """
    Convenience function to chunk text.

    Args:
        text: Text to chunk
        source_url: URL of the source document
        title: Title of the source document
        chunk_size: Size of each text chunk (default from settings)
        chunk_overlap: Overlap between chunks (default from settings)

    Returns:
        List of DocumentChunk objects
    """
    chunker = TextChunker(chunk_size, chunk_overlap)
    return chunker.chunk_text(text, source_url, title)