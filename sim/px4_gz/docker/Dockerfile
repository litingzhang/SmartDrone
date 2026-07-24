ARG BASE_IMAGE=swr.cn-north-4.myhuaweicloud.com/ddn-k8s/docker.io/library/ubuntu:24.04
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive
ENV RUNS_IN_DOCKER=true
ENV USER=smartdrone

COPY ubuntu.sh requirements.txt /tmp/px4-setup/

RUN bash /tmp/px4-setup/ubuntu.sh --no-nuttx \
    && apt-get update -qq \
    && apt-get install -y -qq --no-install-recommends \
        libegl1 \
        libgl1-mesa-dri \
        libgtest-dev \
        mesa-utils \
        python3-matplotlib \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/px4-setup

ENV DEBIAN_FRONTEND=
