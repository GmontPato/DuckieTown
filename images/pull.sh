#!/bin/bash
# Pull all Docker images needed for DuckieTown
set -e

IMAGES=(
    "duckietown/dt-core:daffy-arm64v8"
    "duckietown/dt-ros-commons:v4.3.0-arm64v8"
    "duckietown/dt-device-online:v4.3.0-arm64v8"
    "duckietown/dt-files-api:v4.1.0-arm64v8"
    "duckietown/dt-device-proxy:v4.2.0-arm64v8"
    "duckietown/dt-code-api:v4.1.0-arm64v8"
    "duckietown/dt-device-health:v4.2.1-arm64v8"
    "duckietown/dt-car-interface:v4.1.0-arm64v8"
    "duckietown/dt-duckiebot-interface:v4.3.2-arm64v8"
    "duckietown/dt-device-dashboard:v4.1.0-arm64v8"
    "duckietown/dt-rosbridge-websocket:v4.1.0-arm64v8"
    "duckietown/portainer:daffy-arm64v8"
)

echo "Pulling ${#IMAGES[@]} Docker images..."
for img in "${IMAGES[@]}"; do
    echo "  -> $img"
    docker pull "$img"
done
echo "Done! All images pulled."
