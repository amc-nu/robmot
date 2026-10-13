FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake libopencv-dev libpcl-dev libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /RobMOT
COPY CMakeLists.txt ./
# src/advanced_detection is excluded via .dockerignore (not needed to compile).
COPY src ./src
RUN cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j"$(nproc)"

ENTRYPOINT ["/RobMOT/build/Tracking_3D"]
