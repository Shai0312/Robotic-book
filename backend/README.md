# RAG Knowledge Ingestion Pipeline

A Python-based pipeline for crawling Docusaurus documentation sites, extracting content, generating embeddings using Cohere, and storing them in Qdrant vector database.

## Overview

This pipeline implements the complete workflow for ingesting documentation content:
1. Crawls Docusaurus documentation sites
2. Extracts and cleans text content
3. Chunks text into manageable segments
4. Generates embeddings using Cohere models
5. Stores embeddings in Qdrant vector database for retrieval

## Prerequisites

- Python 3.11+
- Cohere API key
- Qdrant Cloud account and API key

## Setup

1. Clone the repository
2. Navigate to the backend directory: `cd backend`
3. Install dependencies: `pip install -r requirements.txt`
4. Set up environment variables (see Configuration below)

## Configuration

Create a `.env` file in the project root with the following variables:

```env
# Cohere API Configuration
COHERE_API_KEY=your_cohere_api_key_here

# Qdrant Configuration
QDRANT_URL=your_qdrant_cluster_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION_NAME=documents

# Crawler Configuration
CRAWLER_DELAY=1  # Delay in seconds between requests
CRAWLER_MAX_PAGES=1000  # Maximum number of pages to crawl
CRAWLER_TIMEOUT=30  # Request timeout in seconds

# Text Processing Configuration
CHUNK_SIZE=512  # Size of text chunks in characters
CHUNK_OVERLAP=50  # Overlap between chunks in characters

# Logging Configuration
LOG_LEVEL=INFO
```

## Usage

### Command Line Interface

```bash
python src/main.py --urls https://your-docusaurus-site.com/docs
```

Additional options:
- `--chunk-size`: Size of text chunks (default: 512)
- `--chunk-overlap`: Overlap between chunks (default: 50)

### As a Library

```python
from src.main import main_pipeline

success = main_pipeline(
    urls=["https://example.com/docs"],
    chunk_size=512,
    chunk_overlap=50
)
```

## Project Structure

```
backend/
├── src/
│   ├── crawler/           # Web crawling functionality
│   ├── text_processor/    # Text cleaning and chunking
│   ├── embedding/         # Embedding generation with Cohere
│   ├── storage/           # Qdrant vector storage
│   ├── config/            # Configuration management
│   ├── models/            # Data models
│   ├── utils/             # Utility functions
│   └── main.py           # Main pipeline entry point
├── tests/
│   ├── unit/             # Unit tests
│   └── integration/      # Integration tests
├── requirements.txt      # Python dependencies
├── pyproject.toml        # Project metadata
└── .env.example         # Environment variable template
```

## Running Tests

```bash
python -m pytest tests/ -v
```

## Demo

A demo script is available to test the pipeline with sample data:

```bash
python demo.py
```

## Architecture

The pipeline follows a modular architecture with clear separation of concerns:

- **Crawler**: Handles fetching and parsing web pages
- **Text Processor**: Cleans HTML and chunks text content
- **Embedder**: Generates vector embeddings using Cohere
- **Storage**: Manages vector storage in Qdrant
- **Main Pipeline**: Orchestrates the entire workflow

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

[Specify license here]