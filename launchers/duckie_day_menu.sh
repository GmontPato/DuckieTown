#!/bin/bash
# Lanzador del menú de simulación Duckie Day 2026
# Uso: ./launchers/duckie_day_menu.sh

SCRIPT_DIR="$(cd "$(dirname "$0")/.." && pwd)"

echo "Deteniendo contenedores anteriores..."
docker stop duckie_day_menu 2>/dev/null
docker rm duckie_day_menu 2>/dev/null

echo "Ajustando permisos del servidor X..."
xhost +

echo "Iniciando menú Duckie Day 2026..."
docker run -it --rm --name duckie_day_menu \
  --network host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e DELIVERIES_PATH=/data/deliveries.json \
  -e QR_DIR=/data/qr_codes \
  -e CHALLENGES_PATH=/data/challenges.json \
  -e QR_POSITIONS_PATH=/data/qr_positions.json \
  -e SDL_VIDEO_X11_VISUALID= \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$SCRIPT_DIR/scripts":/scripts \
  -v "$SCRIPT_DIR/config":/data \
  -v "$SCRIPT_DIR/logs":/logs \
  --privileged \
  --entrypoint bash \
  duckietown/gym-duckietown:latest \
  -c 'apt-get update -qq && apt-get install -y -qq libfontconfig1 fonts-dejavu-core python3-pip libgl1-mesa-glx 2>/dev/null && cp /data/maps/duckie_day_2026.yaml /gym-duckietown/gym_duckietown/maps/ && sed -i "1539s/if top_down:/if True:/" /gym-duckietown/gym_duckietown/simulator.py && python3 /scripts/duckie_day_menu.py'

echo "Restaurando permisos del servidor X..."
xhost -
