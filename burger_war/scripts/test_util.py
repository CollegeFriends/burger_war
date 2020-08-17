#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import subprocess
import rospy
import time

# Resetをかける
def reset_sim(judge=False):
    try:    
        res1 = subprocess.call("chmod +X burger_war/scripts/*.py",shell=True)    
        res2 = subprocess.call("bash scripts/reset_test.sh",shell=True)        
        # if judge:
        #     subprocess.call("gnome-terminal -e ~/catkin_ws/src/burger_war/judge/test_scripts/init_single_play.sh judge/marker_set/sim.csv localhost:5000 you enemy",shell=True)
        #     subprocess.call("gnome-terminal -e ~/catkin_ws/src/burger_war/judge/test_scripts/set_running.sh localhost:5000",shell=True)                       
        # subprocess.call("roslaunch burger_war your_burger.launch",shell=True)            
    except:
        import traceback
        traceback.print_exc()

        
        
