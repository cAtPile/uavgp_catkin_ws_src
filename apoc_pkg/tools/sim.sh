#!/bin/bash

# 第一个终端：启动PX4 SITL和Gazebo
gnome-terminal --tab --title="PX4 SITL" --command="bash -c 'cd ~/PX4-Autopilot/; make px4_sitl gazebo; exec bash'"

# 第二个终端：启动mavros
gnome-terminal --tab --title="MAVROS" --command="bash -c 'roslaunch mavros px4.launch fcu_url:=\"udp://:14540@127.0.0.1:14557\"; exec bash'"

# 第三个终端：设置catkin工作空间
gnome-terminal --tab --title="Catkin Workspace" --command="bash -c 'cd ~/catkin_ws/; source devel/setup.bash; exec bash'"

echo "PX4仿真环境启动脚本已执行，三个终端已打开。"
