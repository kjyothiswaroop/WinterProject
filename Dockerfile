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
    libglib2.0-0 \
    libsm6 \
    libxrender1 \
    libxext6 \
    libxcb1 \
    libxcb-render0 \
    libxcb-render-util0 \
    libxcb-shape0 \
    libxcb-xfixes0 \
    libxcb-shm0 \
    libxcb-keysyms1 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-randr0 \
    libxcb-sync1 \
    libxcb-xinerama0 \
    libxcb-util1 \
    libxkbcommon-x11-0 \
    libgl1 \
    libglx-mesa0 \
    libgl1-mesa-dri \
    libegl1 \
    libgles2 \
    mesa-utils \
    libqt5gui5 \
    libqt5widgets5 \
    libqt5core5a \
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
