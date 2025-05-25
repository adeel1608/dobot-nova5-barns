FROM python:3.10-slim

WORKDIR /app

# Install dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy the application
COPY services/video-stream /app/services/video-stream

# Set the Python path
ENV PYTHONPATH=/app

# Run the application
CMD ["uvicorn", "services.video-stream.app:app", "--host", "0.0.0.0", "--port", "8000"] 