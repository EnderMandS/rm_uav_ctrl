FROM endermands/ros_noetic_zsh:latest

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO noetic
ARG USERNAME=m

# copy code
ADD src /home/$USERNAME/code/basic_dev/src/
VOLUME /home/$USERNAME/code/basic_dev/src
ADD setup.zsh /
RUN sudo chmod +x /setup.zsh

RUN sudo echo "source /opt/ros/noetic/setup.zsh" >> /home/$USERNAME/.zshrc

# install depend
RUN sudo apt update && \
    sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential && \
    sudo apt install -y libeigen3-dev libsuitesparse-dev libboost-all-dev && \
    sudo apt isntall -y ros-$ROS_DISTRO-foxglove-bridge && \
    sudo rm -rf /var/lib/apt/lists/*

# basic_dev
WORKDIR /home/$USERNAME/code/basic_dev
RUN sudo chmod 777 -R /home/$USERNAME/code/basic_dev
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && catkin_make --only-pkg-with-deps airsim_ros && . devel/setup.sh && catkin_make --only-pkg-with-deps basic_dev

WORKDIR /home/$USERNAME/code

# ORB SLAM3
# RUN git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3 && cd ORB_SLAM3 && \
#     sudo chmod +x build_ros.sh && \
#     sudo echo "export ROS_PACKAGE_PATH=\${ROS_PACKAGE_PATH}:/home/$USERNAME/code/ORB_SLAM3/Examples_old/ROS" >> /home/$USERNAME/.zshrc

USER $USERNAME
ENTRYPOINT [ "/bin/zsh" ]
# ENTRYPOINT [ "/setup.zsh" ]
