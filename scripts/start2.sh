#!/bin/bash


bash scripts/reset.sh

# set judge server state "running"
bash judge/test_scripts/set_running.sh localhost:5000

# launch robot control node
if [ "$1" -eq "1" ]
then
    roslaunch burger_war sim_level_1_cheese.launch
elif [ "$1" -eq "2" ]
then
    roslaunch burger_war sim_level_2_teriyaki.launch
elif [ "$1" -eq "3" ]
then
    roslaunch burger_war sim_level_3_clubhouse.launch
else
    roslaunch burger_war sim_teleop.launch
fi