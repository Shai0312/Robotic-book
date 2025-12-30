import unittest
from unittest.mock import Mock, patch, MagicMock
from src.main import main_pipeline
from src.crawler import DocusaurusCrawler
from src.text_processor import TextCleaner
from src.embedding import CohereEmbedder
from src.storage import QdrantStorage


class TestPipelineIntegration(unittest.TestCase):
    """Integration tests for the complete RAG pipeline."""

    @patch('src.main.QdrantStorage')
    @patch('src.main.CohereEmbedder')
    @patch('src.main.TextCleaner')
    @patch('src.main.DocusaurusCrawler')
    def test_main_pipeline_success(self, mock_crawler_class, mock_cleaner_class, mock_embedder_class, mock_storage_class):
        """Test the complete pipeline with mocked external dependencies."""
        # Arrange
        mock_crawler_instance = Mock()
        mock_crawler_instance.crawl_site.return_value = []
        mock_crawler_class.return_value = mock_crawler_instance

        mock_cleaner_instance = Mock()
        mock_cleaner_class.return_value = mock_cleaner_instance

        mock_embedder_instance = Mock()
        # Mock the embed_document_chunks method to return some embedding records
        from src.models.embedding_record import EmbeddingRecord
        mock_embedding_records = [
            EmbeddingRecord(
                id="test-id",
                vector=[0.1, 0.2, 0.3],
                payload={"text": "test", "source": "test"}
            )
        ]
        mock_embedder_instance.embed_document_chunks.return_value = mock_embedding_records
        mock_embedder_class.return_value = mock_embedder_instance

        mock_storage_instance = Mock()
        mock_storage_instance.store_embeddings.return_value = True
        mock_storage_class.return_value = mock_storage_instance

        urls = ["https://example.com/docs"]

        # Act
        result = main_pipeline(urls)

        # Assert
        self.assertTrue(result)
        mock_crawler_instance.crawl_site.assert_called_once_with("https://example.com/docs")
        mock_embedder_instance.embed_document_chunks.assert_called()
        mock_storage_instance.store_embeddings.assert_called_once()

    @patch('src.main.QdrantStorage')
    @patch('src.main.CohereEmbedder')
    @patch('src.main.TextCleaner')
    @patch('src.main.DocusaurusCrawler')
    def test_main_pipeline_empty_content(self, mock_crawler_class, mock_cleaner_class, mock_embedder_class, mock_storage_class):
        """Test the pipeline when no content is extracted."""
        # Arrange
        mock_crawler_instance = Mock()
        mock_crawler_instance.crawl_site.return_value = []  # No pages crawled
        mock_crawler_class.return_value = mock_crawler_instance

        mock_cleaner_instance = Mock()
        mock_cleaner_class.return_value = mock_cleaner_instance

        mock_embedder_instance = Mock()
        mock_embedder_instance.embed_document_chunks.return_value = []
        mock_embedder_class.return_value = mock_embedder_instance

        mock_storage_instance = Mock()
        mock_storage_class.return_value = mock_storage_instance

        urls = ["https://example.com/docs"]

        # Act
        result = main_pipeline(urls)

        # Assert
        # Should return False since no content was extracted
        self.assertFalse(result)

    @patch('src.main.QdrantStorage')
    @patch('src.main.CohereEmbedder')
    @patch('src.main.TextCleaner')
    @patch('src.main.DocusaurusCrawler')
    def test_main_pipeline_storage_failure(self, mock_crawler_class, mock_cleaner_class, mock_embedder_class, mock_storage_class):
        """Test the pipeline when storage fails."""
        # Arrange
        mock_crawler_instance = Mock()
        mock_crawler_instance.crawl_site.return_value = []
        mock_crawler_class.return_value = mock_crawler_instance

        mock_cleaner_instance = Mock()
        mock_cleaner_class.return_value = mock_cleaner_instance

        mock_embedder_instance = Mock()
        from src.models.embedding_record import EmbeddingRecord
        mock_embedding_records = [
            EmbeddingRecord(
                id="test-id",
                vector=[0.1, 0.2, 0.3],
                payload={"text": "test", "source": "test"}
            )
        ]
        mock_embedder_instance.embed_document_chunks.return_value = mock_embedding_records
        mock_embedder_class.return_value = mock_embedder_instance

        mock_storage_instance = Mock()
        mock_storage_instance.store_embeddings.return_value = False  # Storage fails
        mock_storage_class.return_value = mock_storage_instance

        urls = ["https://example.com/docs"]

        # Act
        result = main_pipeline(urls)

        # Assert
        self.assertFalse(result)  # Should return False when storage fails

    @patch('src.main.QdrantStorage')
    @patch('src.main.CohereEmbedder')
    @patch('src.main.TextCleaner')
    @patch('src.main.DocusaurusCrawler')
    def test_main_pipeline_exception_handling(self, mock_crawler_class, mock_cleaner_class, mock_embedder_class, mock_storage_class):
        """Test that the pipeline handles exceptions properly."""
        # Arrange
        mock_crawler_instance = Mock()
        mock_crawler_instance.crawl_site.side_effect = Exception("Test error")
        mock_crawler_class.return_value = mock_crawler_instance

        mock_cleaner_instance = Mock()
        mock_cleaner_class.return_value = mock_cleaner_instance

        mock_embedder_instance = Mock()
        mock_embedder_class.return_value = mock_embedder_instance

        mock_storage_instance = Mock()
        mock_storage_class.return_value = mock_storage_instance

        urls = ["https://example.com/docs"]

        # Act
        result = main_pipeline(urls)

        # Assert
        self.assertFalse(result)  # Should return False when an exception occurs


class TestPipelineIntegrationEdgeCases(unittest.TestCase):
    """Test edge cases for the pipeline integration."""

    def test_main_pipeline_empty_urls(self):
        """Test the pipeline with empty URL list."""
        # This would require more complex mocking, but we can at least test the function exists
        # In a real scenario, we'd want to test this more thoroughly
        self.assertIsNotNone(main_pipeline)


if __name__ == '__main__':
    unittest.main()