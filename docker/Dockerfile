# syntax=docker/dockerfile:1
ARG BASE_IMAGE=nvcr.io/nvidia/l4t-jetpack:r36.4.0

FROM busybox AS deps
COPY src /src
RUN mkdir -p /deps && cd /src && \
    find . -name package.xml -exec sh -c 'mkdir -p "/deps/$(dirname "$1")" && cp "$1" "/deps/$1"' _ {} \;

FROM ${BASE_IMAGE}
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000
ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8
SHELL ["/bin/bash", "-c"]

RUN if [ ! -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then \
      apt-get update && apt-get install -y --no-install-recommends curl ca-certificates software-properties-common \
      && add-apt-repository -y universe \
      && ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | awk -F\" '{print $4}') \
      && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
      && dpkg -i /tmp/ros2-apt-source.deb && rm /tmp/ros2-apt-source.deb \
      && apt-get update && apt-get install -y --no-install-recommends ros-humble-ros-base \
      && rm -rf /var/lib/apt/lists/*; \
    fi

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-pip \
        build-essential \
        cmake \
        git \
        sudo \
        bash-completion \
        ros-humble-qt-gui-cpp \
        ros-humble-rqt-gui-cpp \
        ros-humble-rqt-graph \
        qtbase5-dev \
        ros-${ROS_DISTRO}-rclcpp \
        ros-${ROS_DISTRO}-rclpy \
        ros-${ROS_DISTRO}-sensor-msgs \
        ros-${ROS_DISTRO}-geometry-msgs \
        ros-${ROS_DISTRO}-tf2-ros \
        ros-${ROS_DISTRO}-cv-bridge \
        ros-${ROS_DISTRO}-realsense2-camera \
        ros-${ROS_DISTRO}-rosidl-default-generators \
        ros-${ROS_DISTRO}-joy \
        ros-${ROS_DISTRO}-teleop-twist-joy \
        ros-${ROS_DISTRO}-teleop-twist-keyboard \
        python3-opencv \
        python3-numpy \
    && rm -rf /var/lib/apt/lists/*

RUN git clone -b v2.4.3 --depth 1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/micro-xrce-dds-agent \
    && mkdir /tmp/micro-xrce-dds-agent/build \
    && cd /tmp/micro-xrce-dds-agent/build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
    && make -j"$(nproc)" \
    && make install \
    && ldconfig \
    && rm -rf /tmp/micro-xrce-dds-agent

RUN if getent passwd ${USER_UID} >/dev/null; then userdel -r "$(getent passwd ${USER_UID} | cut -d: -f1)" || true; fi \
 && if ! getent group ${USER_GID} >/dev/null; then groupadd --gid ${USER_GID} ${USERNAME}; fi \
 && useradd --uid ${USER_UID} --gid ${USER_GID} -G video -m -s /bin/bash ${USERNAME} \
 && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME}

COPY --from=deps /deps /tmp/deps
RUN apt-get update \
 && (rosdep init 2>/dev/null || true) \
 && rosdep update --rosdistro ${ROS_DISTRO} \
 && rosdep install --from-paths /tmp/deps --ignore-src -r -y --rosdistro ${ROS_DISTRO} \
 && rm -rf /tmp/deps /var/lib/apt/lists/*

WORKDIR ..

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER ${USERNAME}
RUN rosdep update --rosdistro ${ROS_DISTRO} \
 && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
 && echo "[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash" >> ~/.bashrc

WORKDIR /ros2_ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
