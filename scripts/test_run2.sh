#!/bin/bash
set -e
set -x
# rosrun burger_war cv_cam2.py &
# ~/anaconda3/envs/CollegeFriends/bin/python burger_war/scripts/localization.py

# ~/anaconda3/envs/CollegeFriends/bin/python burger_war/scripts/test#4.py
bash scripts/reset.sh
bash judge/test_scripts/set_running.sh localhost:5000
roslaunch burger_war your_burger.launch

# gnome-terminal -e "rosrun burger_war cv_cam2.py"
# gnome-terminal -- bash -c "~/anaconda3/envs/CollegeFriends/bin/python burger_war/scripts/localization.py"

# gnome-terminal -- bash -c "~/anaconda3/envs/CollegeFriends/bin/python burger_war/scripts/localization.py"
# gnome-terminal -- bash -c "ls; exec bash"

