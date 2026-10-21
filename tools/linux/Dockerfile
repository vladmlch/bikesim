# Linux aarch64 development lane for the binding graph and Python oracle.
# Build explicitly with tools/linux/container.sh image; never during CMake.
# Base image identity: Docker Hub library/ubuntu:24.04, inspected as metadata
# on 2026-10-21. Actual lane qualification is recorded separately.
FROM ubuntu@sha256:68ee22004a9109b44973313b02ff7b1de250f1de0f5ecb628d73f579aaff33ac

ARG UV_VERSION=0.13.1
ARG UV_SHA256=9041c73c99fdc5892cf128140beb354eca9231d502068cc36e6cb53a8e59ad14
ARG PYTHON_VERSION=3.13.16
ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8

RUN test "$(uname -m)" = aarch64 \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates curl git rsync unzip xz-utils file pkg-config \
        clang clang-18 clang-tidy clang-tidy-18 clang-tools-18 \
        llvm-18 lld-18 g++ g++-13 \
        cmake ninja-build cppcheck shellcheck \
        libwayland-dev wayland-protocols libxkbcommon-dev xorg-dev libgl-dev \
        xvfb xauth libgl1-mesa-dri mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Check the pinned release archive before extracting or executing any bytes.
# uv's pinned managed-Python metadata supplies the interpreter archive hash.
RUN curl --proto '=https' --tlsv1.2 -fLsS \
        "https://releases.astral.sh/github/uv/releases/download/${UV_VERSION}/uv-aarch64-unknown-linux-gnu.tar.gz" \
        -o /tmp/uv.tar.gz \
    && printf '%s  %s\n' "$UV_SHA256" /tmp/uv.tar.gz | sha256sum --check --strict - \
    && tar -xzf /tmp/uv.tar.gz -C /usr/local/bin --strip-components=1 \
        uv-aarch64-unknown-linux-gnu/uv uv-aarch64-unknown-linux-gnu/uvx \
    && rm /tmp/uv.tar.gz \
    && uv python install "$PYTHON_VERSION"

ENV UV_PROJECT_ENVIRONMENT=/opt/bike-venv UV_LINK_MODE=copy
WORKDIR /src
CMD ["sleep", "infinity"]
