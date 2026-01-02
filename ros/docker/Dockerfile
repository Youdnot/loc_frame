FROM python:3.12-slim

WORKDIR /workspace

# Install system dependencies required for OpenCV
RUN apt-get update && apt-get install -y \
    libgl1 \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Copy project metadata files to cache dependencies
COPY pyproject.toml README.md /tmp/

# Install dependencies
RUN pip install --no-cache-dir /tmp/

# Copy the rest of the application code
COPY . .

# Expose the port the app runs on
EXPOSE 8000

# Command to run the application
# CMD ["fastapi", "run", "server.py", "--port", "8000", "--host", "0.0.0.0"]