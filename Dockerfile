FROM ros:lunar-ros-base

RUN apt-get update \
  && DEBIAN_FRONTEND=noninteractive apt-get -y install \
    ros-lunar-cv-bridge \
    ros-lunar-opencv3 \
    ros-lunar-tf \
    python-pip python-matplotlib \
  && rm -rf /var/lib/apt/lists/*
COPY . /kitti2bag
RUN pip install pandas==0.23 fire

WORKDIR /data


