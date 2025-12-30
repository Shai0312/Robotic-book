from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Optional
from src.utils.logger import get_logger
from src.utils.exceptions import StorageException
from src.config.settings import settings
from src.models.embedding_record import EmbeddingRecord

logger = get_logger(__name__)


class QdrantStorage:
    """
    Handles storage and retrieval of embeddings in Qdrant vector database.
    """

    def __init__(self, url: str = None, api_key: str = None, collection_name: str = None):
        """
        Initialize the QdrantStorage.

        Args:
            url: Qdrant URL (if not provided, will use from settings)
            api_key: Qdrant API key (if not provided, will use from settings)
            collection_name: Name of the collection to store embeddings (if not provided, will use from settings)
        """
        self.url = url or settings.QDRANT_URL
        self.api_key = api_key or settings.QDRANT_API_KEY
        self.collection_name = collection_name or settings.QDRANT_COLLECTION_NAME

        # Initialize Qdrant client
        if self.api_key:
            # Handle both cloud and local URLs
            if "https://" in self.url or "http://" in self.url:
                # Cloud instance
                self.client = QdrantClient(
                    url=self.url,
                    api_key=self.api_key,
                    prefer_grpc=False  # Using HTTP for better compatibility
                )
            else:
                # Local instance
                self.client = QdrantClient(
                    host=self.url,
                    api_key=self.api_key,
                    prefer_grpc=False
                )
        else:
            # For local instances without API key
            self.client = QdrantClient(url=self.url)

        # Ensure the collection exists
        self._ensure_collection_exists()

    def _ensure_collection_exists(self, vector_size: int = 1024):
        """
        Ensure the collection exists in Qdrant, create it if it doesn't.

        Args:
            vector_size: Size of the vectors to be stored (default to Cohere's embedding size)
        """
        try:
            # Check if collection exists
            try:
                collection_info = self.client.get_collection(self.collection_name)
                logger.info(f"Collection {self.collection_name} already exists")
                return  # Collection already exists
            except:
                # Collection doesn't exist, so create it
                pass

            # Create collection with appropriate vector configuration
            # Using Cohere's multilingual-v3 model which produces 1024-dim embeddings by default
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=vector_size,
                    distance=models.Distance.COSINE
                )
            )
            logger.info(f"Created new collection: {self.collection_name} with vector size {vector_size}")
        except Exception as e:
            logger.error(f"Error ensuring collection exists: {str(e)}")
            raise StorageException(f"Error ensuring collection exists: {str(e)}")

    def store_embeddings(self, embedding_records: List[EmbeddingRecord]) -> bool:
        """
        Store embedding records in Qdrant.

        Args:
            embedding_records: List of EmbeddingRecord objects to store

        Returns:
            True if storage was successful, False otherwise
        """
        if not embedding_records:
            logger.warning("No embedding records to store")
            return True

        try:
            # Get vector size from the first embedding to ensure correct collection setup
            vector_size = len(embedding_records[0].vector) if embedding_records else 1024

            # Ensure the collection exists with the correct vector size
            self._ensure_collection_exists(vector_size=vector_size)

            # Prepare points for insertion
            points = []
            for record in embedding_records:
                point = models.PointStruct(
                    id=record.id,
                    vector=record.vector,
                    payload=record.payload
                )
                points.append(point)

            # Upload points to Qdrant
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            logger.info(f"Successfully stored {len(embedding_records)} embedding records in Qdrant")
            return True

        except Exception as e:
            logger.error(f"Error storing embeddings in Qdrant: {str(e)}")
            raise StorageException(f"Error storing embeddings in Qdrant: {str(e)}")


    def search_similar(self, query_vector: List[float], limit: int = 10) -> List[EmbeddingRecord]:
        """
        Search for similar embeddings in Qdrant.

        Args:
            query_vector: Vector to search for similar embeddings
            limit: Maximum number of results to return

        Returns:
            List of similar EmbeddingRecord objects
        """
        try:
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=limit
            )

            embedding_records = []
            for result in results:
                record = EmbeddingRecord(
                    id=result.id,
                    vector=result.vector,
                    payload=result.payload
                )
                embedding_records.append(record)

            logger.info(f"Found {len(embedding_records)} similar embeddings")
            return embedding_records

        except Exception as e:
            logger.error(f"Error searching for similar embeddings: {str(e)}")
            raise StorageException(f"Error searching for similar embeddings: {str(e)}")

    def get_embedding_record(self, record_id: str) -> Optional[EmbeddingRecord]:
        """
        Retrieve a specific embedding record by ID.

        Args:
            record_id: ID of the embedding record to retrieve

        Returns:
            EmbeddingRecord object if found, None otherwise
        """
        try:
            records = self.client.retrieve(
                collection_name=self.collection_name,
                ids=[record_id]
            )

            if records:
                record = records[0]
                return EmbeddingRecord(
                    id=record.id,
                    vector=record.vector,
                    payload=record.payload
                )

            return None
        except Exception as e:
            logger.error(f"Error retrieving embedding record {record_id}: {str(e)}")
            raise StorageException(f"Error retrieving embedding record {record_id}: {str(e)}")

    def delete_collection(self) -> bool:
        """
        Delete the entire collection (use with caution).

        Returns:
            True if deletion was successful, False otherwise
        """
        try:
            self.client.delete_collection(self.collection_name)
            logger.info(f"Deleted collection: {self.collection_name}")
            return True
        except Exception as e:
            logger.error(f"Error deleting collection {self.collection_name}: {str(e)}")
            raise StorageException(f"Error deleting collection {self.collection_name}: {str(e)}")


def create_qdrant_storage(url: str = None, api_key: str = None, collection_name: str = None) -> QdrantStorage:
    """
    Convenience function to create a QdrantStorage instance.

    Args:
        url: Qdrant URL (if not provided, will use from settings)
        api_key: Qdrant API key (if not provided, will use from settings)
        collection_name: Name of the collection to store embeddings (if not provided, will use from settings)

    Returns:
        QdrantStorage instance
    """
    return QdrantStorage(url, api_key, collection_name)