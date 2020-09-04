rosnode kill send_id_to_judge
rosnode kill /enemy_bot/send_id_to_judge
rosnode kill /enemy_bot/turtlebot3_teleop_keyboard
rosnode cleanup
rosservice call /gazebo/reset_simulation "{}"
chmod +x burger_controller/scripts/*.py


#!/bin/bash

# set default level 1
VALUE_L="1"
# set default color red
VALUE_SIDE="r"

# get args level setting
while getopts ls: OPT
do
  case $OPT in
    "l" ) FLG_L="TRUE" ; VALUE_L="$OPTARG" ;;
    "s" ) FLG_S="TRUE"  ; VALUE_SIDE="$OPTARG" ;;
  esac
done

# init judge server for sim setting
case $VALUE_SIDE in
    "r" ) bash judge/test_scripts/reset_server.sh judge/marker_set/sim.csv localhost:5000 you enemy ;;
    "b" ) bash judge/test_scripts/reset_server.sh judge/marker_set/sim.csv localhost:5000 enemy you ;;
    * ) bash judge/test_scripts/reset_server.sh judge/marker_set/sim.csv localhost:5000 you enemy ;;
esac

case $VALUE_SIDE in
    "r" ) bash judge/test_scripts/init_single_play.sh judge/marker_set/sim.csv localhost:5000 you enemy ;;
    "b" ) bash judge/test_scripts/init_single_play.sh judge/marker_set/sim.csv localhost:5000 enemy you ;;
    * ) bash judge/test_scripts/init_single_play.sh judge/marker_set/sim.csv localhost:5000 you enemy ;;
esac


# robot
case $VALUE_SIDE in
    "r" ) gnome-terminal -e "roslaunch burger_war restart_sim.launch" ;;
    "b" ) gnome-terminal -e "roslaunch burger_war restart_sim_blue.launch" ;;
    * ) gnome-terminal -e "roslaunch burger_war restart_sim.launch" ;;
esac
