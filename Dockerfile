FROM endermands/ros_noetic_zsh:latest

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO noetic
ARG USERNAME=m

RUN sudo echo "source /opt/ros/noetic/setup.zsh" >> /home/$USERNAME/.zshrc

# install depend
RUN sudo apt update && \
    sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool && \
    sudo apt install -y ninja-build build-essential cmake git vim wget tree pkg-config curl dkms rename g++ gcc unzip && \
    sudo apt install -y libeigen3-dev libsuitesparse-dev libboost-all-dev libgoogle-glog-dev libgflags-dev libgtest-dev && \
    sudo apt install -y ffmpeg libpcl-dev libglew-dev libatlas-base-dev libatlas-base-dev && \
    sudo apt install -y libgtk2.0-dev libtbb-dev libswscale-dev libavdevice-dev libjpeg-dev libpng-dev libtiff5-dev libopenexr-dev && \
    sudo apt install -y ros-$ROS_DISTRO-hector-trajectory-server && \
    sudo rm -rf /var/lib/apt/lists/*

# Ceres
WORKDIR /home/$USERNAME/pkg/ceres
RUN wget https://github.com/ceres-solver/ceres-solver/archive/refs/tags/2.2.0rc3.tar.gz && \
    tar xvf 2.2.0rc3.tar.gz && cd ceres-solver-2.2.0rc3 && mkdir build && cd build \
    cmake -G Ninja .. && ninja && sudo ninja install && ninja clean && \
    rm /home/$USERNAME/pkg/ceres/2.2.0rc3.tar.gz

# g2o
WORKDIR /home/$USERNAME/pkg
RUN git clone --depth 1 https://github.com/RainerKuemmerle/g2o.git g2o && cd g2o && \
    mkdir build && cd build && \
    cmake -G Ninja .. && ninja && sudo ninja install && ninja clean

# OpenCV
WORKDIR /home/$USERNAME/pkg/OpenCV
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip && \
    wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.x.zip && \
    unzip opencv.zip && unzip opencv_contrib.zip && \
    mkdir -p build && cd build && \
    cmake -GNinja -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.x/modules ../opencv-4.x && \
    ninja && sudo ninja install && ninja clean

# clone code
WORKDIR /home/$USERNAME
RUN git clone --depth 1 https://github.com/EnderMandS/rm_uav_ctrl.git UAV && \
    sudo chmod 777 -R /home/$USERNAME/UAV/code/basic_dev && \
    sudo chmod +x /home/$USERNAME/UAV/code/ORBSLAM3/build_ros.sh && \
    sudo echo "export ROS_PACKAGE_PATH=\${ROS_PACKAGE_PATH}:/home/$USERNAME/UAV/code/ORBSLAM3/Examples_old/ROS" >> /home/$USERNAME/.zshrc

# compile
WORKDIR /home/$USERNAME/UAV/code/basic_dev
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && catkin_make --only-pkg-with-deps airsim_ros && . devel/setup.sh && catkin_make --only-pkg-with-deps basic_dev

WORKDIR /home/$USERNAME
VOLUME /home/$USERNAME/UAV

USER $USERNAME
ENTRYPOINT [ "/bin/zsh" ]
# ENTRYPOINT [ "/home/$USERNAME/UAV/setup.zsh" ]
