FROM endermands/ros_noetic_zsh:latest

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO noetic
ARG USERNAME=m

RUN sudo echo "source /opt/ros/noetic/setup.zsh" >> /home/$USERNAME/.zshrc

# install depend
RUN sudo apt update && \
    sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential && \
    sudo apt install -y libeigen3-dev libsuitesparse-dev libboost-all-dev && \
    sudo apt install -y ros-$ROS_DISTRO-foxglove-bridge && \
    sudo rm -rf /var/lib/apt/lists/*

# clone code
COPY . /home/$USERNAME/UAV
RUN sudo chmod 777 -R /home/$USERNAME/UAV/code/basic_dev && \
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
