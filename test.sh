cd ~/catkin_ws/src/burger_war

# gazeboと審判サーバ起動
bash scripts/sim_with_judge.sh
# roslaunch burger_war setup_sim.launch

# 自律走行起動
bash scripts/start.sh -l 3
bash scripts/start2.sh 2

python burger_war/scripts/cv_cam2.py

roslaunch burger_war burger_teleop.launch

bash scripts/reset.sh

# 実行権限
chmod +x [相対パス]

bash scripts/anaconda.sh