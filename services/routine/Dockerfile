# Base image with Python (using slim variant for small size)
FROM python:3.10-slim

# Set working directory
WORKDIR /app

# Install system dependencies if any (e.g., for CV libraries in Validation, but skip for brevity)
# RUN apt-get update && apt-get install -y some-library

# Copy service code
COPY . /app

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Copy the application
COPY services/routine /app/services/routine
COPY config /app/config

# Set the Python path
ENV PYTHONPATH=/app

# Expose the default port (if needed, e.g., 8000)
EXPOSE 8000

# Command to start the FastAPI app using Uvicorn
CMD ["uvicorn", "services.routine.app:app", "--host", "0.0.0.0", "--port", "8000"]
