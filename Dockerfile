FROM endermands/ros_slam:latest

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO noetic
ARG USERNAME=m

# clone code
WORKDIR /home/$USERNAME
RUN git clone --depth 1 https://github.com/EnderMandS/rm_uav_ctrl.git UAV && \
    sudo chmod 777 -R /home/$USERNAME/UAV/code/basic_dev

# fix cv_bridge
# RUN sudo cp /opt/ros/${ROS_DISTRO}/share/cv_bridge/cmake/cv_bridgeConfig.cmake /opt/ros/${ROS_DISTRO}/share/cv_bridge/cmake/cv_bridgeConfig.cmake.backup && \
#     sudo cp /opt/ros/${ROS_DISTRO}/share/cv_bridge/cmake/cv_bridge-extras.cmake /opt/ros/${ROS_DISTRO}/share/cv_bridge/cmake/cv_bridge-extras.cmake.backup && \
#     sudo cp -f /home/$USERNAME/UAV/cv_bridgeConfig.cmake /opt/ros/${ROS_DISTRO}/share/cv_bridge/cmake/cv_bridgeConfig.cmake && \
#     sudo cp -f /home/$USERNAME/UAV/cv_bridge-extras.cmake /opt/ros/${ROS_DISTRO}/share/cv_bridge/cmake/cv_bridge-extras.cmake

# compile
WORKDIR /home/$USERNAME/UAV/code/basic_dev
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && catkin_make --only-pkg-with-deps airsim_ros && . devel/setup.sh && catkin_make --only-pkg-with-deps basic_dev
WORKDIR /home/$USERNAME/UAV/code/orbslam3
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && catkin build

WORKDIR /home/$USERNAME

USER $USERNAME
ENTRYPOINT [ "/bin/zsh" ]
# ENTRYPOINT [ "/home/$USERNAME/UAV/setup.zsh" ]
