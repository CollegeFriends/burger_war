#!/usr/bin/bash
set -e
set -x

# helpを作成
function usage {
  cat <<EOM
Usage: $(basename "$0") [OPTION]...
  -h        Display help
  -e        sim with empty field
  -j        sim with judge  
  -v        sim with versus mode
  Default   sim with marker field (but without enemy)
EOM
  exit 2
}

# ROS関係のセットアップ
# source /opt/ros/kinetic/setup.bash
# source ~/catkin_ws/devel/setup.bash
# export GAZEBO_MODEL_PATH=$HOME/catkin_ws/src/burger_war/burger_war/models/
# export TURTLEBOT3_MODEL=burger

flg_e=""
flg_j=""
flg_v=""
# 通常は、審判なしモードで起動

while getopts ejvh OPT; do
    case "$OPT" in
        e)       
            # -e で何もない空間で起動
            flg_e="TRUE"            
            ;;
        j)
            # -j で審判サーバー有りで起動
            flg_j="TRUE"
            ;;
        v)
            # -v で対戦
            echo HEY
            flg_v="TRUE"
            ;;
        h|*)
            echo $OPT
            usage
            ;;
    esac
done

echo $flg_e
echo $flg_j
echo $flg_v
# robotの対戦環境の作成
if [ $flg_e = "TRUE" ]; then
    # eオプションの時は、マーカーなしで起動 / jオプションは無視
    roslaunch burger_war setup_test_run_empty.launch
else
    if [ $flg_j = "TRUE" ]; then
        # 通常は、マーカー有り・敵有りで起動    
        gnome-terminal -e "python judge/judgeServer.py"
        gnome-terminal -e "python judge/visualizeWindow.py"
        bash judge/test_scripts/init_single_play.sh judge/marker_set/sim.csv localhost:5000 you enemy        
        roslaunch burger_war setup_test_run_enemy.launch
    else
        if [ $flg_v = "TRUE" ]; then
            # 対戦モード
            roslaunch burger_war setup_test_run_enemy.launch
        else
            # 自機だけのテストランモード
            roslaunch burger_war setup_test_run.launch
        fi        
    fi
fi