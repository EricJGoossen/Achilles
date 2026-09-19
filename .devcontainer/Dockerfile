FROM debian:bookworm-slim

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    gdb \
    clang \
    clangd \
    clang-format \
    valgrind \
    ninja-build \
    wget \
    curl \
    ccache \
    libgtest-dev \
    doxygen \
    && rm -rf /var/lib/apt/lists/*

# GLFW (fetched from source by the root CMakeLists.txt, backing
# src/render/ -- see interface::Simulation's own headless flag) needs its
# own X11/OpenGL dev headers to configure; these are build-time only, no
# display required (a headless build/test run never opens a window).
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1-mesa-dev \
    libx11-dev \
    libxrandr-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxi-dev \
    libxext-dev \
    && rm -rf /var/lib/apt/lists/*

# clang-format/clang-tidy are pinned to an explicit LLVM version (via the
# official apt.llvm.org script) rather than whatever "clang-format"/
# "clang-tidy" happen to resolve to in this base image's default apt
# archive -- that's what let local formatting silently disagree with CI's
# ubuntu-latest runner, which has no reason to carry the same default
# version. scripts/check-format.sh and scripts/check-tidy.sh always run
# whichever version update-alternatives points the unversioned name at,
# so CI (.github/workflows/ci.yml) must install the exact same version
# this pins, or the two will drift apart again.
RUN apt-get update \
	&& apt-get install -y --no-install-recommends tree ca-certificates gnupg lsb-release software-properties-common \
	&& wget https://apt.llvm.org/llvm.sh -O /tmp/llvm.sh \
	&& chmod +x /tmp/llvm.sh \
	&& /tmp/llvm.sh 18 \
	&& apt-get install -y --no-install-recommends clang-format-18 clang-tidy-18 \
	&& update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-18 100 \
	&& update-alternatives --install /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-18 100 \
	&& rm -rf /var/lib/apt/lists/* /tmp/llvm.sh

RUN git clone --branch 13.0.0 --depth 1 https://github.com/xtensor-stack/xsimd.git /tmp/xsimd \
	&& cmake -S /tmp/xsimd -B /tmp/xsimd/build -DBUILD_TESTS=OFF \
	&& cmake --install /tmp/xsimd/build \
	&& rm -rf /tmp/xsimd

RUN echo "alias build='cmake --build build -j$(nproc)'" >> ~/.bashrc \
    && echo "alias run='build && ./build/achilles'" >> ~/.bashrc

ENV CC=clang
ENV CXX=clang++