#!/bin/bash
# シミュレーション環境の立ち上げ

# ROS環境変数の設定
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export GAZEBO_MODEL_PATH=$HOME/catkin_ws/src/burger_war/burger_war/models/
export TURTLEBOT3_MODEL=burger
roscd burger_AI

# 審判サーバの立ち上げ
gnome-terminal --tab -e "sh -c 'cd ../;python judge/judgeServer.py'" --tab -e "sh -c 'cd ../;python judge/visualizeWindow.py'"

# 対戦環境の立ち上げ (enemyを省略で敵機有りで起動可能)
roslaunch burger_AI setup.launch enemy:=false