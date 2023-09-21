FROM endermands/ros_noetic_zsh:latest

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO noetic

ADD src /home/m/basic_dev/src/
ADD setup.zsh /
RUN sudo chmod +x /setup.zsh

WORKDIR /home/m/basic_dev
RUN sudo chmod 777 -R /home/m/basic_dev
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && catkin_make --only-pkg-with-deps airsim_ros && . devel/setup.sh && catkin_make --only-pkg-with-deps basic_dev

WORKDIR /home/m/

ENTRYPOINT [ "/bin/zsh" ]
# ENTRYPOINT [ "/setup.zsh" ]
