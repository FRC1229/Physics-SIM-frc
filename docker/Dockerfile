# Ubuntu 24.04 build/run environment for the FRC 1229 physics sim.
# WPILib 2026 vendor libraries (e.g. PhotonLib) need GCC 13's libstdc++,
# which the 22.04 host doesn't have.
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential openjdk-17-jdk-headless ca-certificates git \
        # Sim GUI (GLFW + OpenGL over X11)
        libgl1 libglx-mesa0 libgl1-mesa-dri libegl1 \
        libx11-6 libxrandr2 libxinerama1 libxcursor1 libxi6 libxext6 libxkbcommon0 \
    && rm -rf /var/lib/apt/lists/*

# AdvantageScope (3D field viewer); apt pulls in its Electron dependencies
ARG ADVANTAGESCOPE_VERSION=v26.0.2
ADD https://github.com/Mechanical-Advantage/AdvantageScope/releases/download/${ADVANTAGESCOPE_VERSION}/advantagescope-linux-x64-${ADVANTAGESCOPE_VERSION}.deb /tmp/advantagescope.deb
RUN apt-get update && apt-get install -y --no-install-recommends /tmp/advantagescope.deb \
        libgtk-3-0t64 libnss3 libasound2t64 libgbm1 libxss1 libxshmfence1 fonts-dejavu-core \
    && rm -rf /var/lib/apt/lists/* /tmp/advantagescope.deb

ENV JAVA_HOME=/usr/lib/jvm/java-17-openjdk-amd64
# Gradle cache lives in a host volume so libraries download only once
ENV GRADLE_USER_HOME=/gradle
