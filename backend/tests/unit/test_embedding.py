import unittest
from unittest.mock import Mock, patch, MagicMock
from src.embedding import CohereEmbedder
from src.models.document_chunk import DocumentChunk


class TestCohereEmbedder(unittest.TestCase):
    """Test cases for CohereEmbedder class."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        # Mock the Cohere client to avoid actual API calls
        with patch('src.embedding.cohere_embedder.cohere.Client') as mock_client:
            # Create a mock response for embeddings
            mock_embeddings_response = Mock()
            mock_embeddings_response.embeddings = [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]]

            # Configure the mock client
            mock_instance = Mock()
            mock_instance.embed.return_value = mock_embeddings_response
            mock_client.return_value = mock_instance

            # Create the embedder instance
            self.embedder = CohereEmbedder(api_key="test-key")
            self.embedder.client = mock_instance

    @patch('src.embedding.cohere_embedder.cohere')
    def test_generate_embeddings_basic(self, mock_cohere_module):
        """Test basic embedding generation."""
        # Arrange
        mock_client = Mock()
        mock_response = Mock()
        mock_response.embeddings = [[0.1, 0.2, 0.3]]
        mock_client.embed.return_value = mock_response
        mock_cohere_module.Client.return_value = mock_client

        embedder = CohereEmbedder(api_key="test-key")
        texts = ["Test text"]

        # Act
        result = embedder.generate_embeddings(texts)

        # Assert
        self.assertIsInstance(result, list)
        self.assertEqual(len(result), 1)
        self.assertIsInstance(result[0], list)  # Should be a list of floats

    @patch('src.embedding.cohere_embedder.cohere')
    def test_embed_document_chunks(self, mock_cohere_module):
        """Test embedding document chunks."""
        # Arrange
        mock_client = Mock()
        mock_response = Mock()
        # Return embeddings for two chunks
        mock_response.embeddings = [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]]
        mock_client.embed.return_value = mock_response
        mock_cohere_module.Client.return_value = mock_client

        embedder = CohereEmbedder(api_key="test-key")

        # Create test document chunks
        chunks = [
            DocumentChunk(
                id="chunk1",
                content="Test content 1",
                source_url="https://example.com",
                title="Test Title 1"
            ),
            DocumentChunk(
                id="chunk2",
                content="Test content 2",
                source_url="https://example.com",
                title="Test Title 2"
            )
        ]

        # Act
        result = embedder.embed_document_chunks(chunks)

        # Assert
        self.assertIsInstance(result, list)
        self.assertEqual(len(result), 2)  # Should have embeddings for both chunks

        # Check that the result contains EmbeddingRecord objects
        from src.models.embedding_record import EmbeddingRecord
        for record in result:
            self.assertIsInstance(record, EmbeddingRecord)
            self.assertEqual(len(record.vector), 3)  # Each embedding should have 3 values in this mock

    @patch('src.embedding.cohere_embedder.cohere')
    def test_embed_text_single(self, mock_cohere_module):
        """Test embedding a single text."""
        # Arrange
        mock_client = Mock()
        mock_response = Mock()
        mock_response.embeddings = [[0.1, 0.2, 0.3]]
        mock_client.embed.return_value = mock_response
        mock_cohere_module.Client.return_value = mock_client

        embedder = CohereEmbedder(api_key="test-key")
        text = "Single test text"

        # Act
        result = embedder.embed_text(text)

        # Assert
        self.assertIsInstance(result, list)
        self.assertEqual(len(result), 3)  # Should be a list of 3 floats

    def test_init_without_api_key(self):
        """Test initialization without API key raises error."""
        # Act & Assert
        with self.assertRaises(ValueError):
            CohereEmbedder(api_key="")


class TestCohereEmbedderEdgeCases(unittest.TestCase):
    """Test edge cases for CohereEmbedder class."""

    @patch('src.embedding.cohere_embedder.cohere')
    def test_generate_embeddings_empty_list(self, mock_cohere_module):
        """Test generating embeddings for an empty list."""
        # Arrange
        mock_client = Mock()
        mock_cohere_module.Client.return_value = mock_client

        embedder = CohereEmbedder(api_key="test-key")
        texts = []

        # Act
        result = embedder.generate_embeddings(texts)

        # Assert
        self.assertEqual(result, [])  # Should return empty list

    @patch('src.embedding.cohere_embedder.cohere')
    def test_embed_document_chunks_empty_list(self, mock_cohere_module):
        """Test embedding an empty list of document chunks."""
        # Arrange
        mock_client = Mock()
        mock_cohere_module.Client.return_value = mock_client

        embedder = CohereEmbedder(api_key="test-key")
        chunks = []

        # Act
        result = embedder.embed_document_chunks(chunks)

        # Assert
        self.assertEqual(result, [])  # Should return empty list


if __name__ == '__main__':
    unittest.main()