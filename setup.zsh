#!/bin/zsh
cd /home/m/UAV/code
source devel/setup.zsh
rosrun basic_dev basic_dev
roslaunch ego_planner simple_run_nogui.launch