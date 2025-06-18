FROM python:3.9-slim

WORKDIR /app

# Copy requirements and install dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy shared modules
COPY shared/ /app/shared/

# Copy data directory (contains recipes.json)
COPY data/ /app/data/

# Copy service code
COPY services/api-bridge/ /app/services/api-bridge/

# Set working directory to service
WORKDIR /app/services/api-bridge

# Expose port
EXPOSE 8000

# Run the service
CMD ["python", "app.py"] 