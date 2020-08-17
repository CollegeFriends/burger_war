rosnode kill send_id_to_judge
rosnode kill /enemy_bot/send_id_to_judge
rosnode kill /enemy_bot/turtlebot3_teleop_keyboard
rosnode cleanup
rosservice call /gazebo/reset_simulation "{}"
chmod +x burger_war/scripts/*.py
bash judge/test_scripts/reset_server.sh localhost:5000 you enemy
bash judge/test_scripts/init_single_play.sh judge/marker_set/sim.csv localhost:5000 you enemy
gnome-terminal -e "roslaunch burger_war restart_sim.launch"