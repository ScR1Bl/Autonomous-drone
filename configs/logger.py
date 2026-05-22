"""
Logging module for <ProjectName>.

Provides a centralized logging configuration used across all modules.

Usage:
    At the start of app:
    from src.logging import get_logger
    
    log = get_logger(__name__)

    In modules:
    log = logging.getLogger(__name__) 

Log Levels:
    DEBUG    - detailed diagnostic information
    INFO     - general execution flow
    WARNING  - unexpected but recoverable situations  
    ERROR    - failures that affect execution

Output:
    - Console: WARNING and above
    - File:    logs/app.log, all levels, rotating (max 5MB x 3 backups)

Configuration:
    LOG_LEVEL and LOG_DIR can be overridden.
"""

import logging
import logging.handlers
import sys
from pathlib import Path
from typing import Optional, Union
from datetime import datetime

_configured_loggers: set[str] = set()


def get_logger(
    name: str,
    level: Union[int, str] = logging.INFO,
    log_dir: Union[str, Path] = "logs",
    console: bool = True,
    max_bytes: int = 10 * 1024 * 1024,
    backup_count: int = 5
) -> logging.Logger:
    """
    Configure logging for the application.

    Sets up console and file handlers with appropriate formatters.
    Should be called once at application startup, before any other module
    initializes a logger.

    Args:
        level:   Logging level. One of: DEBUG, INFO, WARNING, ERROR.
                 Defaults to INFO.
        log_dir: Directory where log files will be stored.
                 Created automatically if it does not exist.
                 Defaults to 'logs/'.
        console: If True, also log to console (stderr). Defaults to True.
        max_bytes: Maximum size of log file before rotation (in bytes).
                     Defaults to 10MB.
        backup_count: Number of rotated log files to keep. Defaults to 5.
    """

    logger = logging.getLogger(name)

    if name in _configured_loggers:
        return logger

    logger.setLevel(level)
    logger.propagate = False

    fmt = "[%(asctime)s | %(name)s | %(levelname)s | %(funcName)s]: %(message)s"
    datefmt = "%Y-%m-%d %H:%M:%S"
    formatter = logging.Formatter(fmt=fmt, datefmt=datefmt)

    if console:
        console_handler = logging.StreamHandler(sys.stderr)
        console_handler.setLevel(level)
        console_handler.setFormatter(formatter)
        logger.addHandler(console_handler)


    run_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    log_file = f"run_{run_time}.log"
    log_dir_path = Path(log_dir)
    log_dir_path.mkdir(parents=True, exist_ok=True)
    file_path = log_dir_path / log_file

    file_handler = logging.handlers.RotatingFileHandler(
        file_path,
        maxBytes=max_bytes,
        backupCount=backup_count,
        encoding="utf-8",
    )
    file_handler.setLevel(level)
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)

    _configured_loggers.add(name)
    return logger


def set_global_level(level: Union[int, str]) -> None:
    """Set level for all configured loggers (useful for runtime debugging)."""
    for name in _configured_loggers:
        logging.getLogger(name).setLevel(level)
        for handler in logging.getLogger(name).handlers:
            handler.setLevel(level)


if __name__ == "__main__":
    log = get_logger("test", level="DEBUG")
    log.debug("debug message")
    log.info("info message")
    log.warning("warning message")
    log.error("error message")
    try:
        1 / 0
    except ZeroDivisionError:
        log.exception("caught an exception")