FROM python:3.9-slim

WORKDIR /app

# Copy requirements and install dependencies (service-local)
COPY services/api-bridge/requirements.txt ./requirements.txt
RUN pip install --no-cache-dir -r ./requirements.txt

# Copy shared modules
COPY shared/ /app/shared/

# Copy data directory (contains recipes.json)
COPY data/ /app/data/

# Copy service code
COPY services/api-bridge/ /app/services/api-bridge/

# Set Python path (keep /app as root)
ENV PYTHONPATH=/app

# Expose port
EXPOSE 8000

# Run the service (using module path instead of direct script)
CMD ["python", "-m", "services.api-bridge.app"] 