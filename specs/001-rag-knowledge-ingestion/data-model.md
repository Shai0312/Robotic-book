# Data Model: RAG Knowledge Ingestion

## Entity: DocumentChunk
**Description**: A segment of cleaned text content extracted from documentation pages
- **id**: Unique identifier for the chunk (string)
- **content**: The actual text content of the chunk (string)
- **source_url**: URL of the original documentation page (string)
- **title**: Title of the original documentation page (string)
- **metadata**: Additional metadata about the chunk (dictionary)
- **embedding**: Vector representation of the content (list of floats)
- **created_at**: Timestamp when the chunk was created (datetime)

## Entity: CrawledPage
**Description**: A web page from a Docusaurus site that has been successfully crawled and processed
- **url**: The URL of the page (string)
- **title**: The title of the page (string)
- **content**: The extracted text content (string)
- **html**: The original HTML content (string)
- **status**: Status of the crawl operation (string)
- **crawled_at**: Timestamp when the page was crawled (datetime)
- **processed_content**: Cleaned text content after processing (string)

## Entity: EmbeddingRecord
**Description**: A record containing the embedding vector and associated metadata
- **id**: Unique identifier for the record (string)
- **vector**: The embedding vector (list of floats)
- **payload**: Metadata associated with the vector (dictionary)
  - source_url: URL of the source document
  - content: Original text content
  - title: Title of the document
  - chunk_id: ID of the document chunk
- **created_at**: Timestamp when the record was created (datetime)

## Entity: CrawlSession
**Description**: A session representing a single crawling operation
- **id**: Unique identifier for the session (string)
- **target_url**: The base URL that was crawled (string)
- **status**: Current status of the session (string)
- **total_pages_found**: Number of pages discovered during crawling (integer)
- **successful_pages**: Number of pages successfully processed (integer)
- **start_time**: When the session started (datetime)
- **end_time**: When the session ended (datetime)
- **settings**: Crawl settings used for this session (dictionary)

## Relationships
- One CrawlSession can produce many CrawledPage records
- One CrawledPage can produce many DocumentChunk records
- One DocumentChunk produces one EmbeddingRecord