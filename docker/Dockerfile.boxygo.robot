ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update --allow-releaseinfo-change && \
    apt-get install -y \
        git \
        nano \
        python3-pip \
        python3-vcstool \
        build-essential \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-slam-toolbox \
        ros-humble-robot-localization \
        ros-humble-xacro \
        ros-humble-tf2-ros \
        ros-humble-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*
    
RUN pip3 install --upgrade pip && pip3 install moteus
