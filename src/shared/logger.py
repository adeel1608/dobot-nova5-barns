"""
BARNS Structured Logger with Batched Async Writes to InfluxDB via Telegraf

Usage:
    from shared.logger import log
    
    log("INFO", "Order received", service="oms", order_id="123")
    log("ERROR", "Connection failed", service="scheduler", error="TIMEOUT")
    log("DEBUG", "Processing step", service="routine", cup_id="ORD123-1")
"""

import os
import socket
import time
import threading
import queue
from typing import Optional

# Configuration
DEBUG = os.getenv("DEBUG", "false").lower() == "true"
TELEGRAF_HOST = os.getenv("TELEGRAF_HOST", "telegraf")
TELEGRAF_PORT = int(os.getenv("TELEGRAF_PORT", "8094"))
BATCH_SIZE = 100  # Flush after 100 logs
FLUSH_INTERVAL = 1.0  # Flush every 1 second

# Global state
_log_queue = queue.Queue(maxsize=10000)  # Buffer up to 10k logs
_flush_thread = None
_running = False


def _flush_worker():
    """Background thread that batches and flushes logs to Telegraf via UDP."""
    global _running
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    batch = []
    last_flush = time.time()
    
    while _running:
        try:
            # Try to get a log line (with timeout for periodic flush)
            try:
                log_line = _log_queue.get(timeout=0.1)
                batch.append(log_line)
            except queue.Empty:
                pass
            
            # Flush if batch is full or timeout reached
            now = time.time()
            should_flush = (
                len(batch) >= BATCH_SIZE or 
                (batch and (now - last_flush) >= FLUSH_INTERVAL)
            )
            
            if should_flush and batch:
                try:
                    # Send entire batch as single UDP packet (or multiple if needed)
                    payload = "\n".join(batch).encode('utf-8')
                    
                    # UDP has ~65KB limit, split if necessary
                    if len(payload) > 60000:
                        # Send in chunks
                        for i in range(0, len(batch), 50):
                            chunk = "\n".join(batch[i:i+50]).encode('utf-8')
                            sock.sendto(chunk, (TELEGRAF_HOST, TELEGRAF_PORT))
                    else:
                        sock.sendto(payload, (TELEGRAF_HOST, TELEGRAF_PORT))
                    
                    batch.clear()
                    last_flush = now
                except Exception:
                    # Silently drop logs if Telegraf unavailable (non-blocking)
                    batch.clear()
                    
        except Exception:
            # Continue running even on errors
            pass
    
    sock.close()


def _start_flush_thread():
    """Start the background flush thread (called automatically on first log)."""
    global _flush_thread, _running
    
    if _flush_thread is None or not _flush_thread.is_alive():
        _running = True
        _flush_thread = threading.Thread(target=_flush_worker, daemon=True)
        _flush_thread.start()


def log(level: str, msg: str, service: str = "app", **tags):
    """
    Log a structured message to InfluxDB via Telegraf.
    
    Args:
        level: Log level (INFO, ERROR, DEBUG)
        msg: Short message describing the event
        service: Service name (scheduler, oms, etc.)
        **tags: Additional tags/fields (order_id, cup_id, error, etc.)
    
    Example:
        log("INFO", "Order started", service="oms", order_id="123")
        log("ERROR", "Task failed", service="routine", error="TIMEOUT", cup_id="ORD123-1")
    """
    # Filter debug logs if DEBUG=false
    if not DEBUG and level == "DEBUG":
        return
    
    # Start flush thread on first log
    _start_flush_thread()
    
    # Build InfluxDB line protocol
    # Format: measurement,tag1=val1,tag2=val2 field1="val1",field2="val2" timestamp
    timestamp = int(time.time_ns())
    
    # Tags (indexed) - service and level
    tag_str = f"service={service},level={level}"
    
    # Fields (not indexed) - msg is required, others optional
    fields = [f'msg="{_escape(msg)}"']
    
    for key, value in tags.items():
        if value is not None:
            if isinstance(value, str):
                fields.append(f'{key}="{_escape(value)}"')
            else:
                fields.append(f'{key}={value}')
    
    field_str = ",".join(fields)
    
    # Complete line protocol
    line = f"barns_logs,{tag_str} {field_str} {timestamp}"
    
    # Add to queue (non-blocking - drop if queue full)
    try:
        _log_queue.put_nowait(line)
    except queue.Full:
        # Queue full, drop log silently
        pass


def _escape(s: str) -> str:
    """Escape special characters for InfluxDB line protocol."""
    return s.replace('"', '\\"').replace('\n', '\\n')


def shutdown():
    """Gracefully shutdown the logger (flush remaining logs)."""
    global _running
    _running = False
    if _flush_thread:
        _flush_thread.join(timeout=2)

