FROM osrf/ros:humble-desktop-full

ARG ROS_PACKAGE=ros_base \
    ROS_VERSION=humble

ENV ROS_DISTRO=${ROS_VERSION} \
    ROS_ROOT=/opt/ros/${ROS_VERSION} \
    ROS_PYTHON_VERSION=3 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    DEBIAN_FRONTEND=noninteractive \
    SHELL=/bin/bash

RUN apt-get update \
    && apt-get install -y \
    nano \
    && rm -rf /var/lib/apt/lists/*

ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config
    
# Set up sudo
RUN apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Only on server
# Add vglusers group (GID must match host!)
# ARG VGL_GID=1001
# RUN groupadd -g $VGL_GID vglusers || true \
#     && usermod -aG vglusers $USERNAME

RUN apt-get update \
    && apt-get install -y jstest-gtk\
    usbutils \
    xxd \
    net-tools \
    iputils-ping \
    fzf \
    python3-pip \
    python3-scipy \
    wget \
    libgeographic-dev \
    mesa-utils \
    && rm -rf /ver/lib/apt/lists/*

# Download ROS2 packages
# ros-$ROS_DISTRO-diagnostic-updater jest wymieniony jawnie (a nie tylko dociągany
# tranzytywnie przez robot-localization), bo w praktyce zdarzało się, że apt dobierał
# do niego starszą wersję niż ta, z którą został zbudowany robot_localization, co
# kończyło się błędem "libdiagnostic_updater.so: cannot open shared object file"
# przy starcie ekf_node. Jawne żądanie najnowszej wersji w tej samej transakcji apt
# ogranicza ryzyko takiego rozjazdu.
RUN apt-get update \
    && apt-get install -y \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher-gui\
    ros-$ROS_DISTRO-tf-transformations \
    ros-$ROS_DISTRO-diagnostic-updater \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-imu-tools \
    ros-$ROS_DISTRO-plotjuggler-ros \
    ros-$ROS_DISTRO-py-trees \
    ros-$ROS_DISTRO-py-trees-ros \
    ros-$ROS_DISTRO-octomap-server \
    ros-$ROS_DISTRO-octomap-msgs \
    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-teleop-twist-joy

USER root

RUN git clone https://github.com/kimushun1101/ros2-aliases.git /home/dev/ros2-aliases 
COPY docker/ros2aliases.env /home/dev/ros2-aliases/.env

# install NVIDIA software
RUN apt-get update \
    && apt-get -y --quiet --no-install-recommends install \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
    && rm -rf /var/lib/apt/lists/*

# only on server
# RUN apt-get update \
#     && apt-get -y --quiet --no-install-recommends install \
#     libnvidia-gl-535 \
#     && rm -rf /var/lib/apt/lists/*


ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV QT_X11_NO_MITSHM=1

# Install stonefish library

RUN apt-get update && apt-get install -y \
    python3 python3-pip python3-venv \
    git cmake build-essential \
    libopencv-dev libglm-dev libsdl2-dev libfreetype6-dev doxygen \
    libx11-dev x11-apps \
    && rm -rf /var/lib/apt/lists/*

RUN sed -i 's/-lSDL2 /-lSDL2/' /usr/lib/x86_64-linux-gnu/cmake/SDL2/sdl2-config.cmake

RUN git clone --branch agh-marines-branch https://github.com/AGH-Marines/stonefish.git  /home/$USERNAME/stonefish \
    && cd /home/$USERNAME/stonefish \
    && mkdir build \ 
    && cd build \
    && cmake .. \
    && make -j$(nproc) \
    && make install \
    && export LD_LIBRARY_PATH=/home/$USERNAME/stonefish/build/libStonefish.so:$LD_LIBRARY_PATH

RUN mkdir -p /home/$USERNAME/ros2_ws
RUN chown -R ${USERNAME}:root /home/${USERNAME}/ros2_ws

# Clone packages for developement

## Dualshock 4 driver and package
RUN sudo pip3 install ds4drv
RUN sudo pip3 install osqp
RUN sudo pip3 install casadi

## Download uard
RUN sudo pip3 install pyserial

USER ${USERNAME}

USER root

WORKDIR /home/$USERNAME/ros2_ws/src


RUN echo 'source /opt/ros/'$ROS_DISTRO'/setup.bash' >> /home/$USERNAME/.bashrc \
    && echo 'source /home/'$USERNAME'/ros2-aliases/ros2_aliases.bash' >> /home/$USERNAME/.bashrc \
    && echo 'source /home/'$USERNAME'/ros2_ws/install/setup.bash' >> /home/$USERNAME/.bashrc


# Install crush

RUN sudo mkdir -p /etc/apt/keyrings \
    && curl -fsSL https://repo.charm.sh/apt/gpg.key | sudo gpg --dearmor -o /etc/apt/keyrings/charm.gpg \
    && echo "deb [signed-by=/etc/apt/keyrings/charm.gpg] https://repo.charm.sh/apt/ * *" | sudo tee /etc/apt/sources.list.d/charm.list \
    && sudo apt update && sudo apt install crush

COPY docker/entrypoint.sh /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"]