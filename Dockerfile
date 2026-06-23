ARG BASE_IMAGE=duckietown/dt-core:daffy-amd64
FROM ${BASE_IMAGE}

ARG LAUNCHER=duckie_day_solution
ENV LAUNCHER=${LAUNCHER}
ENV DELIVERIES_PATH=/data/deliveries.json

RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys F42ED6FBAB17C654 \
    && apt-get update && apt-get install -y --no-install-recommends \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

COPY packages /code/catkin_ws/src
COPY config /data
COPY launchers /launchers

RUN chmod +x /code/catkin_ws/src/duckie_day_solution/src/*.py /launchers/*.sh \
    && /bin/bash -lc "source /opt/ros/noetic/setup.bash && cd /code/catkin_ws && catkin build"

ENTRYPOINT ["/bin/bash", "-lc"]
CMD ["/launchers/${LAUNCHER}.sh"]
