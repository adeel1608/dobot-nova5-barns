# Base image with Python (using slim variant for small size)
FROM python:3.10-slim

# Set working directory
WORKDIR /app

# Install system dependencies if any (for automation equipment drivers)
# RUN apt-get update && apt-get install -y some-automation-library

# Copy service code
COPY services/automation /app/services/automation

# Install Python dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Set the Python path
ENV PYTHONPATH=/app

# Expose the default port
EXPOSE 8000

# Command to start the FastAPI app using Uvicorn
CMD ["uvicorn", "services.automation.app:app", "--host", "0.0.0.0", "--port", "8000"] 