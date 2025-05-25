FROM ros:foxy-ros-base   # hypothetical base with ROS
WORKDIR /app
COPY . /app
# Install Python deps (including grpcio)
RUN apt-get update && apt-get install -y python3-pip && pip3 install -r requirements.txt
# Expose gRPC port
EXPOSE 50051
CMD ["python3", "server.py"]
