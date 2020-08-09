#!/bin/bash

# ROS関係のセットアップ
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export GAZEBO_MODEL_PATH=$HOME/catkin_ws/src/burger_war/burger_war/models/
export TURTLEBOT3_MODEL=burger

# 指定したPythonファイルを実行

# rosrun burger_war cv_cam2.py &
# ~/anaconda3/envs/CollegeFriends/bin/python burger_war/scripts/controlBot.py
chmod +X ./burger_war/scripts/*.py
bash scripts/reset.sh
rosrun burger_war controlBot.py
# python burger_war/scripts/print3.py
# & ~/anaconda3/envs/CollegeFriends/bin/python burger_war/scripts/localization.py


# gnome-terminal -e "rosrun burger_war cv_cam2.py"
# gnome-terminal -- bash -c "~/anaconda3/envs/CollegeFriends/bin/python burger_war/scripts/localization.py"

# gnome-terminal -- bash -c "~/anaconda3/envs/CollegeFriends/bin/python burger_war/scripts/localization.py"
# gnome-terminal -- bash -c "ls; exec bash"

