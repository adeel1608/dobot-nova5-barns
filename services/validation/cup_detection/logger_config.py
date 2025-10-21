"""
Logging Configuration Module
Provides JSON file logging with rotation for warnings and errors (configurable).
"""

from __future__ import annotations

import json
import logging
import logging.handlers
import os
import sys
import traceback
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Dict, Optional


class JSONFormatter(logging.Formatter):
    """
    Custom formatter to output logs as robust JSON.

    Shape:
      {
        "time": "2025-10-12T05:13:37Z",
        "level": "ERROR",
        "logger": "my.module",
        "msg": "Something went wrong",
        "ctx": {"rtsp_url": "...", "attempts": 3},   # optional
        "error": "Traceback (most recent call last):\n ..."  # optional
      }
    """

    def formatTime(self, record: logging.LogRecord) -> str:
        # ISO-8601 in UTC with trailing Z (e.g., 2025-10-12T05:13:37.123Z)
        dt = datetime.fromtimestamp(record.created, tz=timezone.utc)
        # Keep milliseconds only when present to reduce noise
        iso = dt.isoformat(timespec="milliseconds").replace("+00:00", "Z")
        return iso

    def formatException(self, ei) -> str:
        # Use Python's traceback to produce a readable single string
        return "".join(traceback.format_exception(*ei)).rstrip()

    def format(self, record: logging.LogRecord) -> str:
        # Let logging populate message etc.
        message = record.getMessage()

        entry: Dict[str, Any] = {
            "time": self.formatTime(record),
            "level": record.levelname,
            "logger": record.name,
            "msg": message,
        }

        # Attach structured context if present
        extra_ctx = getattr(record, "extra_data", None)
        if extra_ctx:
            # Ensure context is JSON-serializable
            entry["ctx"] = extra_ctx

        # Attach exception information if present
        if record.exc_info:
            entry["error"] = self.formatException(record.exc_info)
        elif record.exc_text:
            entry["error"] = record.exc_text

        # Dump as JSON with UTF-8 friendly output and safe fallback
        try:
            return json.dumps(entry, ensure_ascii=False, default=str)
        except Exception:
            # As a last resort, stringify non-serializable bits
            safe_entry = {k: (str(v) if not isinstance(v, (str, int, float, bool, type(None), dict, list)) else v)
                          for k, v in entry.items()}
            return json.dumps(safe_entry, ensure_ascii=False, default=str)


@dataclass
class LoggerConfig:
    """
    Container for logging settings.
    """
    log_file: str = os.path.join("logs", "app.log")
    level: int = logging.WARNING
    max_bytes: int = 5 * 1024 * 1024  # 5 MB
    backup_count: int = 5
    json_formatter: Optional[logging.Formatter] = None
    # Target logger name; using a named logger reduces root-handler surprises
    logger_name: str = "app"

    def ensure_dir(self) -> None:
        directory = os.path.dirname(os.path.abspath(self.log_file)) or "."
        os.makedirs(directory, exist_ok=True)


def _find_existing_rotating_handler(logger: logging.Logger, filepath: str) -> Optional[logging.Handler]:
    """
    Return an existing RotatingFileHandler bound to the same filepath if any.
    """
    target = os.path.abspath(filepath)
    for h in logger.handlers:
        if isinstance(h, logging.handlers.RotatingFileHandler):
            try:
                if os.path.abspath(getattr(h, "baseFilename", "")) == target:
                    return h
            except Exception:
                # Some handlers may not expose baseFilename
                continue
    return None


def setup_logging(
    log_file: Optional[str] = None,
    level: Optional[int] = None,
    max_bytes: Optional[int] = None,
    backup_count: Optional[int] = None,
    logger_name: Optional[str] = None,
) -> logging.Logger:
    """
    Initialize a rotating JSON file logger. Idempotent: calling again won't duplicate handlers.

    Args:
        log_file: path to log file. If None, uses LoggerConfig default.
        level: logging level (e.g., logging.INFO). If None, uses default.
        max_bytes: rotation threshold in bytes. If None, uses default.
        backup_count: number of rotated files to keep. If None, uses default.
        logger_name: which named logger to configure. If None, uses default ("app").

    Returns:
        The configured logger.
    """
    cfg = LoggerConfig()

    if log_file is not None:
        cfg.log_file = log_file
    if level is not None:
        cfg.level = level
    if max_bytes is not None:
        cfg.max_bytes = max_bytes
    if backup_count is not None:
        cfg.backup_count = backup_count
    if logger_name is not None:
        cfg.logger_name = logger_name

    cfg.ensure_dir()

    logger = logging.getLogger(cfg.logger_name)
    logger.setLevel(cfg.level)

    # Prevent log messages from duplicating via root handlers
    logger.propagate = False

    # If an appropriate handler already exists, just update its level/formatter and return
    existing = _find_existing_rotating_handler(logger, cfg.log_file)
    if existing:
        existing.setLevel(cfg.level)
        existing.setFormatter(cfg.json_formatter or JSONFormatter())
        return logger

    # Create a new rotating file handler with UTF-8 encoding
    handler = logging.handlers.RotatingFileHandler(
        cfg.log_file, maxBytes=cfg.max_bytes, backupCount=cfg.backup_count, encoding="utf-8"
    )
    handler.setLevel(cfg.level)
    handler.setFormatter(cfg.json_formatter or JSONFormatter())

    logger.addHandler(handler)

    # (Optional) also mirror WARNING+ to stderr in dev environments
    # Comment out if not desired in production.
    if sys.stderr and not any(isinstance(h, logging.StreamHandler) for h in logger.handlers if h.stream is sys.stderr):
        stderr_handler = logging.StreamHandler(stream=sys.stderr)
        stderr_handler.setLevel(cfg.level)
        stderr_handler.setFormatter(cfg.json_formatter or JSONFormatter())
        logger.addHandler(stderr_handler)

    return logger


def get_logger(name: Optional[str] = None) -> logging.Logger:
    """
    Get a child logger that inherits the same handlers/level.
    If name is None, returns the base "app" logger.
    """
    base = logging.getLogger("app")
    if name in (None, "", "app"):
        return base
    return base.getChild(name)


# ---------------------------
# Convenience helper functions
# ---------------------------

def log_with_context(logger: logging.Logger, level: int, message: str, **kwargs: Any) -> None:
    """
    Log a message with structured context (kwargs) merged under 'ctx'.
    Use:
        log_with_context(logger, logging.ERROR, "Camera failed", rtsp_url=..., attempts=3)
    """
    # Attach kwargs in a separate field consumed by JSONFormatter
    logger.log(level, message, extra={"extra_data": kwargs})


def log_warning(logger: logging.Logger, message: str, **kwargs: Any) -> None:
    log_with_context(logger, logging.WARNING, message, **kwargs)


def log_error(logger: logging.Logger, message: str, **kwargs: Any) -> None:
    """
    If you want traceback capture, call with exc_info=True:
        try:
            ...
        except Exception:
            log_error(logger, "Failed to connect", rtsp_url=url, exc_info=True)
    """
    # Pull exc_info out of kwargs if present so Logger handles it natively
    exc_info = kwargs.pop("exc_info", None)
    logger.log(logging.ERROR, message, extra={"extra_data": kwargs}, exc_info=exc_info)


def log_camera_error(logger: logging.Logger, message: str, **kwargs: Any) -> None:
    """
    Semantic sugar for camera/RTSP issues. Example keys: rtsp_url, attempts, timeout_s.
    """
    log_error(logger, message, **kwargs)


def log_detection_error(logger: logging.Logger, message: str, frame_count: Optional[int] = None, **kwargs: Any) -> None:
    """
    Log detection-related errors with optional frame_count and arbitrary context.
    """
    context = dict(kwargs)
    if frame_count is not None:
        context["frame_count"] = frame_count
    log_error(logger, message, **context)


def log_connection_status(logger, status: Dict[str, Any]) -> None:
    log_warning(
        logger,
        f"Connection status: {status.get('status', 'Unknown')}",
        connected=bool(status.get("connected", False)),
        attempts=int(status.get("attempts", 0) or 0),
        # removed frame_count
    )
