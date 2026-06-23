ARG BASE_IMAGE=duckietown/dt-core:daffy-amd64
FROM ${BASE_IMAGE}

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*
