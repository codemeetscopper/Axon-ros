"""Structured logging helper."""
from __future__ import annotations

import logging
from typing import Any, Dict


def get_logger(name: str) -> logging.Logger:
    """Return a configured standard logger."""
    logger = logging.getLogger(name)
    if not logger.handlers:
        handler = logging.StreamHandler()
        formatter = logging.Formatter(
            fmt="%(asctime)s %(name)s %(levelname)s %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
        )
        handler.setFormatter(formatter)
        logger.addHandler(handler)
        logger.setLevel(logging.INFO)
    return logger


def log_structured(logger: logging.Logger, message: str, **fields: Dict[str, Any]) -> None:
    """Log a message with structured fields for debugging."""
    if fields:
        logger.info("%s | %s", message, fields)
    else:
        logger.info(message)
