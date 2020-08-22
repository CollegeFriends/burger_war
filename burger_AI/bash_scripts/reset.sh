# simのリセット

# 審判サーバへの送信をしているnodeを切る
rosnode kill send_id_to_judge
rosnode kill /enemy_bot/send_id_to_judge
rosnode kill /enemy_bot/turtlebot3_teleop_keyboard
# gazeboのリセット
rosnode cleanup
rosservice call /gazebo/reset_simulation "{}"
# 審判サーバのリセット
roscd burger_war
cd ../
bash judge/test_scripts/reset_server.sh localhost:5000 you enemy
bash judge/test_scripts/init_single_play.sh judge/marker_set/sim.csv localhost:5000 you enemy