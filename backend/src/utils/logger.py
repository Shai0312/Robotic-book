import logging
from typing import Optional
from src.config.settings import settings


def setup_logger(name: str, level: Optional[str] = None) -> logging.Logger:
    """
    Set up a structured logger with the specified name and level.

    Args:
        name: Name of the logger
        level: Logging level (e.g., 'DEBUG', 'INFO', 'WARNING', 'ERROR')

    Returns:
        Configured logger instance
    """
    if level is None:
        level = settings.LOG_LEVEL

    # Convert string level to logging constant
    log_level = getattr(logging, level.upper(), logging.INFO)

    logger = logging.getLogger(name)
    logger.setLevel(log_level)

    # Avoid adding multiple handlers if logger already exists
    if not logger.handlers:
        handler = logging.StreamHandler()
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        handler.setFormatter(formatter)
        logger.addHandler(handler)

    return logger


def get_logger(name: str) -> logging.Logger:
    """
    Get a logger instance with the specified name.

    Args:
        name: Name of the logger

    Returns:
        Logger instance
    """
    return setup_logger(name)