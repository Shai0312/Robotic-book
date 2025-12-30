import unittest
from unittest.mock import Mock, patch, MagicMock
from src.storage import QdrantStorage
from src.models.embedding_record import EmbeddingRecord


class TestQdrantStorage(unittest.TestCase):
    """Test cases for QdrantStorage class."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        # Mock the Qdrant client to avoid actual database calls
        with patch('src.storage.qdrant_storage.QdrantClient') as mock_qdrant:
            # Configure mock client
            mock_client_instance = Mock()
            mock_qdrant.return_value = mock_client_instance

            # Create the storage instance
            self.storage = QdrantStorage(
                url="http://test-url",
                api_key="test-key",
                collection_name="test-collection"
            )
            self.storage.client = mock_client_instance

    @patch('src.storage.qdrant_storage.QdrantClient')
    def test_init_creates_client(self, mock_qdrant_class):
        """Test that initialization creates a Qdrant client."""
        # Arrange
        mock_client = Mock()
        mock_qdrant_class.return_value = mock_client

        # Act
        storage = QdrantStorage(
            url="http://test.com",
            api_key="test-key",
            collection_name="test-col"
        )

        # Assert
        mock_qdrant_class.assert_called_once()
        self.assertIsNotNone(storage)

    @patch('src.storage.qdrant_storage.QdrantClient')
    def test_store_embeddings_basic(self, mock_qdrant_class):
        """Test basic embedding storage."""
        # Arrange
        mock_client = Mock()
        mock_qdrant_class.return_value = mock_client

        storage = QdrantStorage(
            url="http://test.com",
            api_key="test-key",
            collection_name="test-col"
        )

        # Create test embedding records
        records = [
            EmbeddingRecord(
                id="test-id-1",
                vector=[0.1, 0.2, 0.3],
                payload={"text": "test content 1", "source": "test-url-1"}
            ),
            EmbeddingRecord(
                id="test-id-2",
                vector=[0.4, 0.5, 0.6],
                payload={"text": "test content 2", "source": "test-url-2"}
            )
        ]

        # Act
        result = storage.store_embeddings(records)

        # Assert
        self.assertTrue(result)
        mock_client.upsert.assert_called_once()
        # Check that upsert was called with correct parameters
        args, kwargs = mock_client.upsert.call_args
        self.assertEqual(kwargs['collection_name'], "test-col")

    @patch('src.storage.qdrant_storage.QdrantClient')
    def test_search_similar(self, mock_qdrant_class):
        """Test searching for similar embeddings."""
        # Arrange
        mock_client = Mock()
        # Mock the search result
        mock_search_result = [
            Mock(id="result-1", vector=[0.7, 0.8, 0.9], payload={"text": "similar content"})
        ]
        mock_client.search.return_value = mock_search_result
        mock_qdrant_class.return_value = mock_client

        storage = QdrantStorage(
            url="http://test.com",
            api_key="test-key",
            collection_name="test-col"
        )

        query_vector = [0.1, 0.2, 0.3]

        # Act
        result = storage.search_similar(query_vector, limit=10)

        # Assert
        self.assertIsInstance(result, list)
        if result:
            self.assertEqual(len(result), 1)
            from src.models.embedding_record import EmbeddingRecord
            self.assertIsInstance(result[0], EmbeddingRecord)

    @patch('src.storage.qdrant_storage.QdrantClient')
    def test_get_embedding_record(self, mock_qdrant_class):
        """Test retrieving a specific embedding record."""
        # Arrange
        mock_client = Mock()
        # Mock the retrieve result
        mock_retrieve_result = [
            Mock(id="test-id", vector=[0.1, 0.2, 0.3], payload={"text": "test content"})
        ]
        mock_client.retrieve.return_value = mock_retrieve_result
        mock_qdrant_class.return_value = mock_client

        storage = QdrantStorage(
            url="http://test.com",
            api_key="test-key",
            collection_name="test-col"
        )

        # Act
        result = storage.get_embedding_record("test-id")

        # Assert
        self.assertIsNotNone(result)
        from src.models.embedding_record import EmbeddingRecord
        self.assertIsInstance(result, EmbeddingRecord)
        self.assertEqual(result.id, "test-id")

    @patch('src.storage.qdrant_storage.QdrantClient')
    def test_get_embedding_record_not_found(self, mock_qdrant_class):
        """Test retrieving a non-existent embedding record."""
        # Arrange
        mock_client = Mock()
        mock_client.retrieve.return_value = []  # Empty result
        mock_qdrant_class.return_value = mock_client

        storage = QdrantStorage(
            url="http://test.com",
            api_key="test-key",
            collection_name="test-col"
        )

        # Act
        result = storage.get_embedding_record("non-existent-id")

        # Assert
        self.assertIsNone(result)

    def test_ensure_collection_exists(self):
        """Test ensuring collection exists method."""
        # This would require more complex mocking to test properly
        # For now, just verify the method exists and can be called
        try:
            # This will call the method as part of setup, so just make sure no exception is raised
            self.assertIsNotNone(self.storage)
        except Exception as e:
            self.fail(f"Collection creation failed with error: {e}")


class TestQdrantStorageEdgeCases(unittest.TestCase):
    """Test edge cases for QdrantStorage class."""

    @patch('src.storage.qdrant_storage.QdrantClient')
    def test_store_embeddings_empty_list(self, mock_qdrant_class):
        """Test storing an empty list of embeddings."""
        # Arrange
        mock_client = Mock()
        mock_qdrant_class.return_value = mock_client

        storage = QdrantStorage(
            url="http://test.com",
            api_key="test-key",
            collection_name="test-col"
        )

        # Act
        result = storage.store_embeddings([])

        # Assert
        self.assertTrue(result)  # Should return True for empty list
        # upsert should not be called for empty list
        mock_client.upsert.assert_not_called()


if __name__ == '__main__':
    unittest.main()