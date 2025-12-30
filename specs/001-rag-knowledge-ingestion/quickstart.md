# Quickstart: RAG Knowledge Ingestion Pipeline

## Prerequisites
- Python 3.11+
- pip package manager
- Cohere API key
- Qdrant Cloud account and API key

## Setup

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Create and activate virtual environment**
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. **Install dependencies**
   ```bash
   pip install -r requirements.txt
   ```

4. **Set up environment variables**
   ```bash
   cp .env.example .env
   # Edit .env with your Cohere and Qdrant credentials
   ```

## Configuration

1. **Environment Variables**
   - `COHERE_API_KEY`: Your Cohere API key for embedding generation
   - `QDRANT_URL`: Your Qdrant Cloud cluster URL
   - `QDRANT_API_KEY`: Your Qdrant API key
   - `QDRANT_COLLECTION_NAME`: Name of the collection to store embeddings (default: "documents")

2. **Pipeline Configuration**
   - URLs to crawl are specified in the main function or via command line arguments
   - Chunk size and overlap settings can be configured in the settings module

## Usage

1. **Run the full pipeline**
   ```bash
   cd backend
   python src/main.py --url https://your-docusaurus-site.com
   ```

2. **Customize parameters**
   ```bash
   python src/main.py --url https://your-site.com --chunk-size 512 --chunk-overlap 50
   ```

## Pipeline Components

1. **Crawling**: Fetches and parses Docusaurus documentation pages
2. **Text Processing**: Cleans HTML, extracts content, and chunks text
3. **Embedding**: Generates vector embeddings using Cohere models
4. **Storage**: Stores embeddings and metadata in Qdrant vector database

## Verification

After running the pipeline, verify:
- Documents were successfully crawled and processed
- Embeddings were generated without errors
- Records were stored in Qdrant collection
- Test queries return relevant results