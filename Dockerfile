FROM ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive
ARG GTSAM_VERSION=4.2a5

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    cmake \
    git \
    libboost-all-dev \
    libeigen3-dev \
    libmetis-dev \
    libopencv-dev \
    pkg-config \
 && rm -rf /var/lib/apt/lists/*

RUN git clone --branch ${GTSAM_VERSION} --depth 1 https://github.com/borglab/gtsam.git /tmp/gtsam \
 && cmake -S /tmp/gtsam -B /tmp/gtsam/build \
    -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_USE_SYSTEM_EIGEN=ON \
    -DGTSAM_BUILD_TESTS=OFF \
   -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
   -DGTSAM_BUILD_UNSTABLE=OFF \
   -DGTSAM_BUILD_PYTHON=OFF \
 && cmake --build /tmp/gtsam/build -j"$(nproc)" \
 && cmake --install /tmp/gtsam/build \
 && rm -rf /tmp/gtsam

WORKDIR /opt/vicon2gt
COPY . .

RUN cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DVICON2GT_BUILD_ROS=OFF \
 && cmake --build build --target estimate_vicon2gt_csv -j"$(nproc)"

ENV LD_LIBRARY_PATH=/usr/local/lib
WORKDIR /work

ENTRYPOINT ["/opt/vicon2gt/build/estimate_vicon2gt_csv"]
CMD ["--help"]