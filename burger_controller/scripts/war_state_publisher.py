#!/usr/bin/env python
# -*- coding: utf-8 -*-

# respect judge/visualizeConsole.py
# respect all_sensor_sample.py

import rospy
from burger_war.msg import war_state

import requests
from time import sleep
import json

class WarStateBot(object):
    def __init__(self, mySide, displayLog = False):
        self.side = mySide
        self.displayLog = displayLog
        self.state = war_state()        
        self.war_state_pub = rospy.Publisher('war_state',war_state,queue_size=1)

    def fetchWarState(self):        
        resp = requests.get("http://localhost:5000/warState")        
        resp_json = resp.json()
        self.state.time = resp_json['time']       
        self.state.my_side = self.side         
        self.state.my_point = resp_json['scores'][self.side]        
        self.state.target_names = []
        self.state.target_owner = []
        self.state.target_point = []

        for target in resp_json['targets']:            
            self.state.target_names.append(target['name'])
            self.state.target_owner.append(target['player'])                        
            self.state.target_point.append(int(target['point']))
            
        if self.side == 'r':
            self.state.enemy_point = resp_json['scores']['b']        
        elif self.side == 'b':
            self.state.enemy_point = resp_json['scores']['r']        
        else:            
            rospy.logerr("UNEXPECTED SIDE NAME : {}".format(self.side))


    def strategy(self):        
        # rospy.loginfo("MY SIDE IS {}".format(self.side))

        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            # fetch war_state
            self.fetchWarState()
            if self.displayLog:                
                rospy.loginfo(self.state.time)
                rospy.loginfo(self.state.my_point-self.state.enemy_point)                        
            # publish war_state topic
            self.war_state_pub.publish(self.state)

            r.sleep()

if __name__ == '__main__':    
    mySide = rospy.get_param("side", default="b")        
    rospy.init_node('war_state')        
    bot = WarStateBot(mySide = mySide, displayLog=False)    
    bot.strategy()