"""Utilities for timestamped logging output in the panel."""

from __future__ import annotations

from datetime import datetime


def timestamped_line(message: str) -> str:
    """Return *message* prefixed with the current local timestamp."""

    timestamp = datetime.now().isoformat(timespec="seconds")
    return f"[{timestamp}] {message}"
