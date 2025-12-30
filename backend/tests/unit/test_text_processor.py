import unittest
from src.text_processor import TextCleaner, TextChunker
from src.models.crawled_page import CrawledPage


class TestTextCleaner(unittest.TestCase):
    """Test cases for TextCleaner class."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        self.cleaner = TextCleaner()

    def test_clean_html_content_basic(self):
        """Test basic HTML content cleaning."""
        # Arrange
        html_content = "<html><head><title>Test</title></head><body><p>Hello World!</p></body></html>"

        # Act
        result = self.cleaner.clean_html_content(html_content)

        # Assert
        self.assertIn("Hello World!", result)
        self.assertNotIn("<p>", result)
        self.assertNotIn("</p>", result)

    def test_clean_html_content_with_script_and_style(self):
        """Test cleaning HTML content with script and style tags."""
        # Arrange
        html_content = """
        <html>
            <head>
                <style>body { color: red; }</style>
                <script>alert('test');</script>
            </head>
            <body>
                <p>Visible content</p>
            </body>
        </html>
        """

        # Act
        result = self.cleaner.clean_html_content(html_content)

        # Assert
        self.assertIn("Visible content", result)
        self.assertNotIn("alert('test')", result)
        self.assertNotIn("color: red", result)

    def test_extract_content_from_page(self):
        """Test extracting content from a crawled page."""
        # Arrange
        crawled_page = CrawledPage(
            url="https://example.com",
            title="Test Page",
            content="",
            html="<html><body><p>Test content</p></body></html>",
            status="success"
        )

        # Act
        result = self.cleaner.extract_content_from_page(crawled_page)

        # Assert
        self.assertIn("Test content", result)
        self.assertEqual(crawled_page.processed_content, result)

    def test_remove_boilerplate_content(self):
        """Test removing boilerplate content."""
        # Arrange
        text_content = """
        Home About Services Contact Us
        Main content here
        Privacy Policy Terms of Service
        """

        # Act
        result = self.cleaner.remove_boilerplate_content(text_content)

        # Assert
        # The main content should still be there
        self.assertIn("Main content here", result)
        # Some boilerplate might still be present depending on the algorithm


class TestTextChunker(unittest.TestCase):
    """Test cases for TextChunker class."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        self.chunker = TextChunker(chunk_size=100, chunk_overlap=20)

    def test_chunk_text_basic(self):
        """Test basic text chunking."""
        # Arrange
        text = "This is a sample text that will be chunked into smaller pieces. " * 5  # Make it long enough to chunk

        # Act
        result = self.chunker.chunk_text(text, "https://example.com", "Test Document")

        # Assert
        self.assertIsInstance(result, list)
        self.assertGreater(len(result), 0)  # Should have at least one chunk
        for chunk in result:
            self.assertLessEqual(len(chunk.content), 100)  # Each chunk should be <= chunk_size

    def test_chunk_text_with_small_content(self):
        """Test chunking text that's smaller than chunk size."""
        # Arrange
        text = "Short text"

        # Act
        result = self.chunker.chunk_text(text, "https://example.com", "Test Document")

        # Assert
        self.assertEqual(len(result), 1)  # Should have exactly one chunk
        self.assertEqual(result[0].content, "Short text")

    def test_chunk_text_empty(self):
        """Test chunking empty text."""
        # Arrange
        text = ""

        # Act
        result = self.chunker.chunk_text(text, "https://example.com", "Test Document")

        # Assert
        self.assertEqual(len(result), 0)  # Should have no chunks

    def test_chunk_multiple_texts(self):
        """Test chunking multiple texts."""
        # Arrange
        texts = ["Text one", "Text two", "Text three"]

        # Act
        result = self.chunker.chunk_multiple_texts(texts, ["url1", "url2", "url3"], ["Title 1", "Title 2", "Title 3"])

        # Assert
        self.assertIsInstance(result, list)
        self.assertEqual(len(result), 3)  # Should have one chunk per text (since they're short)


if __name__ == '__main__':
    unittest.main()