# The sentry co-simulation's container: Achilles's devcontainer image plus what
# the hosted aruw-mcb build (scons) and Project Otto (uv, pyrealsense2 and
# PyGObject's native libraries) need. scripts/run-sentry-sim.sh builds it on top
# of .devcontainer/Dockerfile's image and runs the sim, the MCB and Otto in one
# container: Otto's devices are the sim's pseudo-terminals.
ARG BASE=achilles-dev
FROM ${BASE}

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    pkg-config \
    procps \
    libusb-1.0-0 \
    libgirepository1.0-dev \
    libcairo2-dev \
    gir1.2-gstreamer-1.0 \
    libgl1-mesa-dri \
    libglx-mesa0 \
    && rm -rf /var/lib/apt/lists/*

# aruw-mcb's own devcontainer pins the same scons.
RUN pip3 install --break-system-packages "scons==4.0.1" jinja2 pyelftools requests

COPY --from=ghcr.io/astral-sh/uv:0.11.28 /uv /uvx /usr/local/bin/

# The container runs as the host's user (so nothing in the checkouts ends up
# owned by root), who needs a home and git's trust in the mounted checkouts.
RUN mkdir -m 777 /home/dev && git config --system --add safe.directory '*'
ENV HOME=/home/dev
