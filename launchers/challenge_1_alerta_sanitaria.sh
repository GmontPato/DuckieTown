#!/bin/bash
# Lanzador Desafio 1: Alerta Sanitaria (Clasificatoria)
# Uso: ./launchers/challenge_1_alerta_sanitaria.sh

SCRIPT_DIR="$(cd "$(dirname "$0")/.." && pwd)"

echo "Deteniendo contenedores anteriores..."
docker stop duckie_day_challenge 2>/dev/null
docker rm duckie_day_challenge 2>/dev/null

echo "Iniciando Desafio 1: Alerta Sanitaria..."
docker run -it --rm --name duckie_day_challenge \
  --network host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e QR_DIR=/data/qr_codes \
  -e CHALLENGES_PATH=/data/challenges.json \
  -e QR_POSITIONS_PATH=/data/qr_positions.json \
  -e SDL_VIDEO_X11_VISUALID= \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$SCRIPT_DIR/scripts":/scripts \
  -v "$SCRIPT_DIR/config":/data \
  --privileged \
  --entrypoint bash \
  duckietown/gym-duckietown:latest \
  -c 'apt-get update -qq && apt-get install -y -qq libfontconfig1 fonts-dejavu-core 2>/dev/null && cp /data/maps/duckie_day_2026.yaml /gym-duckietown/gym_duckietown/maps/ && sed -i "1539s/if top_down:/if True:/" /gym-duckietown/gym_duckietown/simulator.py && python3 /scripts/duckie_day_challenge.py 1'
