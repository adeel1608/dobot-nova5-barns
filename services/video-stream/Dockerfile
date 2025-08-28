FROM python:3.10-slim-bookworm

WORKDIR /app

# Install system dependencies for OpenCV and camera access
RUN apt-get update && apt-get install -y \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    libgomp1 \
    libgstreamer1.0-0 \
    libgstreamer-plugins-base1.0-0 \
    libgtk-3-0 \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libavutil-dev \
    v4l-utils \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# Install dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy the application
COPY services/video-stream /app/services/video-stream

# Set the Python path
ENV PYTHONPATH=/app

# Run the application
CMD ["uvicorn", "app:app", "--app-dir", "/app/services/video-stream", "--host", "0.0.0.0", "--port", "8000"]