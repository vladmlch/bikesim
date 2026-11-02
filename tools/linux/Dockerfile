# Linux aarch64 development lane for the native root graph.
# Build explicitly with tools/linux/container.sh image; never during CMake.
# Base image identity: Docker Hub library/ubuntu:24.04, inspected as metadata
# on 2026-10-21. Actual lane qualification is recorded separately.
# The driver supplies a narrow context containing only this Dockerfile, the
# committed dependency lock and the acquisition/extraction scripts. No host environment is copied.
FROM ubuntu@sha256:68ee22004a9109b44973313b02ff7b1de250f1de0f5ecb628d73f579aaff33ac AS acquire
ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8
RUN test "$(uname -m)" = aarch64 \
    && apt-get update \
    && apt-get install -y --no-install-recommends ca-certificates curl jq unzip xz-utils \
    && rm -rf /var/lib/apt/lists/*
COPY dependencies.lock.json /tmp/bike-dependencies.lock.json
COPY acquire_deps.sh extract_deps.sh /tmp/
RUN bash /tmp/acquire_deps.sh /tmp/bike-dependencies.lock.json /opt/bike-deps linux-aarch64

FROM ubuntu@sha256:68ee22004a9109b44973313b02ff7b1de250f1de0f5ecb628d73f579aaff33ac AS lane

ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8

RUN test "$(uname -m)" = aarch64 \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates curl git jq rsync unzip xz-utils file pkg-config \
        clang clang-18 clang-tidy clang-tidy-18 clang-tools-18 \
        llvm-18 lld-18 libclang-rt-18-dev g++ g++-13 \
        cmake make ninja-build cppcheck shellcheck \
        libwayland-dev wayland-protocols libxkbcommon-dev xorg-dev libgl-dev \
        xvfb xauth libgl1-mesa-dri mesa-utils \
    && rm -rf /var/lib/apt/lists/*

COPY --from=acquire /opt/bike-deps /opt/bike-deps
WORKDIR /src
CMD ["sleep", "infinity"]
