#!/bin/bash
set -e
set -x


# ROS関係のセットアップ
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export GAZEBO_MODEL_PATH=$HOME/catkin_ws/src/burger_war/burger_war/models/
export TURTLEBOT3_MODEL=burger


# 通常は、審判なしモードで起動
# " bash setup_test -j " で審判サーバー有りで起動
while getopts ":j" opts
do
    case $opts in
        j)
            echo j
            bash scripts/reset.sh
            set judge server state "running"
            bash judge/test_scripts/set_running.sh localhost:5000
    esac
done

# robotの対戦環境の作成
roslaunch burger_war setup_test_run.launch