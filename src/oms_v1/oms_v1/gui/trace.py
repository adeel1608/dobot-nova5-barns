"""
gui/trace.py

Mirrors the runtime-trace style used in
oms_v1/sequences/espresso.py and paper_cups.py:

    [SCOPE] message ...

so that GUI activity blends into the existing production logs and is easy to
correlate with sequence traces during debugging. A bounded ring buffer is also
maintained in process so the Streamlit UI can render the last N lines.
"""

from __future__ import annotations

import sys
import threading
import time
from collections import deque
from typing import Any, Deque, Iterable, Tuple


GUI_TRACE_DEBUG = True

_BUFFER_LIMIT = 2000
_buffer_lock = threading.Lock()
_buffer: Deque[Tuple[float, str, str]] = deque(maxlen=_BUFFER_LIMIT)


def _format_value(value: Any, max_len: int = 240) -> str:
    """Compact printable representation for trace logs."""
    try:
        text = repr(value)
    except Exception:
        text = f"<{type(value).__name__}>"
    if len(text) > max_len:
        text = text[: max_len - 3] + "..."
    return text


def gui_log(scope: str, message: str) -> None:
    """Print a trace line to stdout and append to the in-memory ring buffer.

    The print mirrors espresso.py's `_trace_step` so a single grep over the
    robot logs surfaces both sequence and GUI activity.
    """
    line = f"[{scope}] {message}"
    if GUI_TRACE_DEBUG:
        try:
            print(line, flush=True)
        except Exception:
            pass
    try:
        with _buffer_lock:
            _buffer.append((time.time(), scope, message))
    except Exception:
        pass


def gui_log_error(scope: str, message: str, exc: BaseException | None = None) -> None:
    """Trace an error path. Always printed regardless of GUI_TRACE_DEBUG."""
    line = f"[{scope}] ERROR {message}"
    if exc is not None:
        line += f" exc={type(exc).__name__}: {exc}"
    try:
        print(line, file=sys.stderr, flush=True)
    except Exception:
        pass
    with _buffer_lock:
        _buffer.append((time.time(), scope, f"ERROR {message}"))


def snapshot(limit: int | None = None) -> list[Tuple[float, str, str]]:
    """Return a copy of the most recent trace lines (oldest first)."""
    with _buffer_lock:
        items = list(_buffer)
    if limit is not None and limit > 0:
        items = items[-limit:]
    return items


def render_lines(items: Iterable[Tuple[float, str, str]]) -> list[str]:
    """Render snapshot tuples into human-readable text lines."""
    out: list[str] = []
    for ts, scope, message in items:
        stamp = time.strftime("%H:%M:%S", time.localtime(ts))
        out.append(f"{stamp} [{scope}] {message}")
    return out


def clear() -> None:
    """Clear the in-memory ring buffer (does not affect stdout history)."""
    with _buffer_lock:
        _buffer.clear()
