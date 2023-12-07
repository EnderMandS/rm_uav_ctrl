FROM endermands/ros_slam:latest

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO noetic
ARG USERNAME=m

# clone code
COPY . /home/$USERNAME/UAV
RUN sudo chmod -R 777 /home/$USERNAME/UAV

# compile
WORKDIR /home/$USERNAME/UAV/code/
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && catkin_make --only-pkg-with-deps airsim_ros && \
    . devel/setup.sh && catkin_make -DCATKIN_WHITELIST_PACKAGES="" -DCMAKE_EXPORT_COMPILE_COMMANDS=1

USER $USERNAME
ENTRYPOINT [ "/home/m/UAV/setup.zsh" ]
