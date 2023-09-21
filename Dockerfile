FROM endermands/ros_noetic_zsh:latest

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO noetic
ARG USERNAME=m

ADD src /home/$USERNAME/basic_dev/src/
ADD setup.zsh /
RUN sudo chmod +x /setup.zsh

WORKDIR /home/$USERNAME/basic_dev
RUN sudo chmod 777 -R /home/$USERNAME/basic_dev
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && catkin_make --only-pkg-with-deps airsim_ros && . devel/setup.sh && catkin_make --only-pkg-with-deps basic_dev

WORKDIR /home/$USERNAME/
VOLUME /home/$USERNAME/basic_dev/src

USER $USERNAME
ENTRYPOINT [ "/bin/zsh" ]
# ENTRYPOINT [ "/setup.zsh" ]
