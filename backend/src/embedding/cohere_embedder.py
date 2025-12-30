import cohere
from typing import List, Union
from src.utils.logger import get_logger
from src.utils.exceptions import EmbeddingException, RateLimitException
from src.config.settings import settings
from src.models.document_chunk import DocumentChunk
from src.models.embedding_record import EmbeddingRecord

logger = get_logger(__name__)


class CohereEmbedder:
    """
    Generates embeddings using Cohere's API.
    """

    def __init__(self, api_key: str = None):
        """
        Initialize the CohereEmbedder.

        Args:
            api_key: Cohere API key (if not provided, will use from settings)
        """
        api_key = api_key or settings.COHERE_API_KEY
        if not api_key:
            raise ValueError("Cohere API key is required")

        self.client = cohere.Client(api_key)
        self.model = "embed-multilingual-v3.0"  # Using a good general-purpose model

    def generate_embeddings(self, texts: List[str], batch_size: int = 96) -> List[List[float]]:
        """
        Generate embeddings for a list of texts.

        Args:
            texts: List of texts to embed
            batch_size: Number of texts to process in each batch (Cohere API limit is 96)

        Returns:
            List of embedding vectors (each vector is a list of floats)
        """
        if not texts:
            return []

        all_embeddings = []

        # Process in batches to respect API limits
        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]
            try:
                logger.info(f"Generating embeddings for batch {i//batch_size + 1}/{(len(texts)-1)//batch_size + 1}")

                response = self.client.embed(
                    texts=batch,
                    model=self.model,
                    input_type="search_document"  # Using search_document for knowledge base content
                )

                batch_embeddings = [embedding for embedding in response.embeddings]
                all_embeddings.extend(batch_embeddings)

            except cohere.CohereAPIError as e:
                logger.error(f"Cohere API error: {str(e)}")
                if "Too Many Requests" in str(e) or e.status_code == 429:
                    raise RateLimitException(f"Rate limit exceeded: {str(e)}")
                else:
                    raise EmbeddingException(f"Error generating embeddings: {str(e)}")
            except Exception as e:
                logger.error(f"Error generating embeddings: {str(e)}")
                raise EmbeddingException(f"Error generating embeddings: {str(e)}")

        logger.info(f"Generated embeddings for {len(texts)} texts")
        return all_embeddings

    def embed_document_chunks(self, chunks: List[DocumentChunk]) -> List[EmbeddingRecord]:
        """
        Generate embeddings for a list of DocumentChunk objects.

        Args:
            chunks: List of DocumentChunk objects to embed

        Returns:
            List of EmbeddingRecord objects with embeddings and metadata
        """
        if not chunks:
            return []

        # Extract text content from chunks
        texts = [chunk.content for chunk in chunks]

        # Generate embeddings
        embeddings = self.generate_embeddings(texts)

        # Create EmbeddingRecord objects
        embedding_records = []
        for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
            payload = {
                "source_url": chunk.source_url,
                "content": chunk.content,
                "title": chunk.title,
                "chunk_id": chunk.id,
                "metadata": chunk.metadata
            }

            embedding_record = EmbeddingRecord(
                id=chunk.id,  # Using the same ID as the original chunk
                vector=embedding,
                payload=payload
            )

            embedding_records.append(embedding_record)

        logger.info(f"Created {len(embedding_records)} embedding records")
        return embedding_records

    def embed_text(self, text: str) -> List[float]:
        """
        Generate embedding for a single text.

        Args:
            text: Text to embed

        Returns:
            Embedding vector (list of floats)
        """
        embeddings = self.generate_embeddings([text])
        return embeddings[0] if embeddings else []


def create_cohere_embedder(api_key: str = None) -> CohereEmbedder:
    """
    Convenience function to create a CohereEmbedder instance.

    Args:
        api_key: Cohere API key (if not provided, will use from settings)

    Returns:
        CohereEmbedder instance
    """
    return CohereEmbedder(api_key)