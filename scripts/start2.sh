#!/bin/bash

bash scripts/reset.sh

# set judge server state "running"
bash judge/test_scripts/set_running.sh localhost:5000

# launch robot control node
roslaunch burger_war sim_robot_run2.launch enemy_level:=$1