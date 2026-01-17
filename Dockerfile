FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# ----------------------------
# Base system packages
# ----------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    sudo \
    curl \
    wget \
    git \
    vim \
    nano \
    less \
    tzdata \
    locales \
    python3 \
    python3-pip \
    python3-venv \
    build-essential \
    cmake \
    pkg-config \
    bash-completion \
    libeigen3-dev \
 && rm -rf /var/lib/apt/lists/*

# ----------------------------
# Locale
# ----------------------------
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

# ----------------------------
# Python venv (PEP 668-safe)
# ----------------------------
RUN python3 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

# Upgrade python tooling (prevents easy_install errors)
RUN pip install --upgrade pip setuptools wheel build

# ----------------------------
# User (Ubuntu-like)
# ----------------------------
ARG USERNAME=ubuntu
ARG UID=1000
ARG GID=1000

RUN if ! getent group ${GID}; then \
        groupadd -g ${GID} ${USERNAME}; \
    fi \
 && if ! id -u ${USERNAME} >/dev/null 2>&1; then \
        useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USERNAME}; \
    fi \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
WORKDIR /home/${USERNAME}

# ----------------------------
# CycloneDDS defaults (edit path if needed)
# ----------------------------
ENV CYCLONEDDS_HOME=/home/ubuntu/cyclonedds/install
ENV CMAKE_PREFIX_PATH=${CYCLONEDDS_HOME}
ENV LD_LIBRARY_PATH=${CYCLONEDDS_HOME}/lib

CMD ["bash"]
