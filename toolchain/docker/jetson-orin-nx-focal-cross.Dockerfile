ARG BASE_IMAGE=ubuntu:20.04
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    ca-certificates \
    cmake \
    ccache \
    g++ \
    g++-aarch64-linux-gnu \
    gcc \
    gcc-aarch64-linux-gnu \
    git \
    make \
    ninja-build \
    pkg-config \
    python3 \
    rsync \
 && rm -rf /var/lib/apt/lists/*
