FROM endermands/ros_slam:latest

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO noetic
ARG USERNAME=m

# clone code
WORKDIR /home/$USERNAME
RUN git clone --depth 1 https://github.com/EnderMandS/rm_uav_ctrl.git UAV
    # sudo chmod 777 -R /home/$USERNAME/UAV/code/src/basic_dev

# compile
WORKDIR /home/$USERNAME/UAV/code/
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && catkin_make --only-pkg-with-deps airsim_ros && . devel/setup.sh && catkin_make --only-pkg-with-deps basic_dev \
    . /opt/ros/${ROS_DISTRO}/setup.sh && catkin_make --only-pkg-with-deps ego_planner uav_simulator -DCMAKE_BUILD_TYPE=Release
# Could not build through git action
# WORKDIR /home/$USERNAME/UAV
# RUN . /opt/ros/${ROS_DISTRO}/setup.sh && catkin build

WORKDIR /home/$USERNAME/UAV

USER $USERNAME
ENTRYPOINT [ "/bin/zsh" ]
# ENTRYPOINT [ "/home/$USERNAME/UAV/setup.zsh" ]
