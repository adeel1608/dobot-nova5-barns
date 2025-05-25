import datetime
import sys

# Open the log file in write mode (overwrites existing content) to start fresh each run.
log_file = open('coffee_log.txt', 'w')

def log(message: str):
    """Log a message with a timestamp to both console and log file."""
    timestamp = datetime.datetime.now().strftime("%H:%M:%S")
    full_message = f"[{timestamp}] {message}"
    # Write to console
    print(full_message)
    # Write to log file
    log_file.write(full_message + "\n")
    log_file.flush()

def close():
    """Close the log file (to ensure all data is written)."""
    log_file.close()
