#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Resetをかける
import os
import subprocess
try:    
    res1 = subprocess.call("chmod +X burger_war/scripts/*.py",shell=True)
    res2 = subprocess.call("bash scripts/reset_test.sh",shell=True)
    # print(res1,res2)
    # print("is Success? : ",res1 == res2 == True)
except:
    import traceback
    traceback.print_exc()

# controlBotを起動
from controlBot import ControlBot
import rospy
rospy.init_node('control_node')    
bot = ControlBot()
reward = bot.strategy()
print(reward)
