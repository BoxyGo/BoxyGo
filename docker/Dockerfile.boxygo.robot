ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update --allow-releaseinfo-change && \
    apt-get install -y \
        git \
        nano \
        swig \
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
        ros-humble-isaac-ros-visual-slam \
        ros-humble-isaac-ros-realsense \
        ros-humble-isaac-ros-examples \
        ros-humble-isaac-ros-nvblox \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep update && rosdep install isaac_ros_nvblox
    
RUN pip3 install --upgrade pip && pip3 install moteus

RUN mkdir -p /usr/local/bin/scripts/entrypoint_additions

COPY additional_entrypoint.sh /usr/local/bin/scripts/entrypoint_additions/additional_entrypoint.sh
RUN chmod +x /usr/local/bin/scripts/entrypoint_additions/additional_entrypoint.sh
