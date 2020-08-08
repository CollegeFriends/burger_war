# rosnode kill send_id_to_judge
# rosnode kill /enemy_bot/send_id_to_judge
# rosnode kill /enemy_bot/turtlebot3_teleop_keyboard



# ROS関係のセットアップ
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export GAZEBO_MODEL_PATH=$HOME/catkin_ws/src/burger_war/burger_war/models/
export TURTLEBOT3_MODEL=burger


rosnode cleanup
rosservice call /gazebo/reset_simulation "{}"
# bash judge/test_scripts/reset_server.sh localhost:5000 you enemy
# bash judge/test_scripts/init_single_play.sh judge/marker_set/sim.csv localhost:5000 you enemy
# roslaunch burger_war setup_test_run.launch
# gnome-terminal -e "roslaunch burger_war restart_sim.launch"