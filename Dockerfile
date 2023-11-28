FROM endermands/ros_slam:latest

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO noetic
ARG USERNAME=m

# clone code
WORKDIR /home/$USERNAME
RUN git clone --depth 1 -b commit https://github.com/EnderMandS/rm_uav_ctrl.git UAV

# compile
WORKDIR /home/$USERNAME/UAV/code/
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && catkin_make --only-pkg-with-deps airsim_ros && \
    . devel/setup.sh && catkin_make --only-pkg-with-deps basic_dev && \
    catkin_make -DCATKIN_WHITELIST_PACKAGES="" -DCMAKE_EXPORT_COMPILE_COMMANDS=1

WORKDIR /home/$USERNAME/UAV

USER $USERNAME
# ENTRYPOINT [ "/bin/zsh" ]
ENTRYPOINT [ "/home/m/UAV/setup.zsh" ]
