#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
from burger_war.msg import war_state

target_map = {
    "FriedShrimp_N":    [0.0,       +0.35/2.0],
    "FriedShrimp_S":    [0.0,       -0.35/2.0],
    "FriedShrimp_E":    [+0.35/2.0, 0.0],
    "FriedShrimp_W":    [-0.35/2.0, 0.0],
    "Omelette_N":       [+0.53,     +0.53+0.15/2.0],
    "Omelette_S":       [+0.53,     +0.53-0.15/2.0],
    "Tomato_N":         [-0.53,     +0.53+0.15/2.0],
    "Tomato_S":         [-0.53,     +0.53-0.15/2.0],
    "OctopusWiener_N":  [+0.53,     -0.53+0.15/2.0],
    "OctopusWiener_S":  [+0.53,     -0.53-0.15/2.0],
    "Pudding_N":        [-0.53,     -0.53+0.15/2.0],
    "Pudding_S":        [-0.53,     -0.53-0.15/2.0],
}

for key in target_map.keys():
    if key[-1] == "N":
        target_map[key][1] += 0.3
    if key[-1] == "S":
        target_map[key][1] -= 0.3
    if key[-1] == "E":
        target_map[key][0] += 0.3
    if key[-1] == "W":
        target_map[key][0] -= 0.3



point_map = {
"00":  [-0.8,+0.7],
"01":  [-0.2,+0.7],
"02":  [+0.2,+0.7],
"03":  [+0.8,+0.7],

"10":  [-0.8,+0.25],
"11":  [-0.3,+0.3],
"12":  [+0.3,+0.3],
"13":  [+0.8,+0.25],

"20":  [-0.8,-0.25],
"21":  [-0.3,-0.3],
"22":  [+0.3,-0.3],
"23":  [+0.8,-0.25],

"30":  [-0.8,-0.7],
"31":  [-0.2,-0.7],
"32":  [+0.2,-0.7],
"33":  [+0.8,-0.7]}


def getPoint(r,c):
    ret = None
    try:
        global point_map        
        key = str(r) + str(c)        
        ret = point_map[key]        
    except:
        pass
    return ret

distance = 0.3  # magic parm of distance to the target

def getNearestTarget(x,y, war_state):
    if war_state is None:        
        return None

    global target_map
    war_state_dict = converter(war_state)
    dist = None
    nearest_target_name = None
    for key in target_map:
        tmp = (target_map[key][0] - x) ** 2 + (target_map[key][1] - y) ** 2
        if dist is None or (war_state_dict[key]['owner'] != war_state.my_side and tmp < dist):
            nearest_target_name = key
            dist = tmp
    return nearest_target_name

def converter(war_state):
    war_state_dict = {}
    for i in range(len(war_state.target_names)):
        war_state_i = {}
        war_state_i['owner'] = war_state.target_owner[i]
        war_state_i['point'] = war_state.target_point[i]
        war_state_dict[war_state.target_names[i]] = war_state_i
    return war_state_dict

def getTargetPose(name):
    global target_map
    xy = target_map[name]
    if name[-1] == "S":
        th = math.pi / 2.0
    elif name[-1] == "N":
        th = -math.pi / 2.0
    elif name[-1] == "E":
        th = math.pi
    elif name[-1] == "W":
        th = 0.0
    else:
        raise Exception

    if name in ["Tomato_N","Omelette_N"]:
        row = [0]
    elif name in ["FriedShrimp_N","FriedShrimp_E","FriedShrimp_W","FriedShrimp_S",
                    "Tomato_S","Omelette_S","Pudding_N","OctopusWiener_N"]:
        row = [1,2]
    elif name in ["Pudding_S","OctopusWiener_S"]:
        row = [3]
    else:
        raise Exception

    if name in ["Tomato_N","Tomato_S",
                "FriedShrimp_W",
                "Pudding_N","Pudding_S"]:
        col = [0,1]
    elif name in ["FriedShrimp_N","FriedShrimp_S"]:
        col = [1,2]
    elif name in ["Omelette_N","Omelette_S",
                    "FriedShrimp_W",
                    "OctopusWiener_N","OctopusWiener_S"]:
        col = [2,3]        
    return xy,th, row, col

def getRC(x,y):
    
    global target_map    
    dist = None
    nearest_target_name = None

    for key in target_map:
        tmp = (target_map[key][0] - x) ** 2 + (target_map[key][1] - y) ** 2
        if dist is None or tmp < dist:
            nearest_target_name = key
            dist = tmp
    nearest_target_name
    xy,_,row,col = getTargetPose(nearest_target_name)

    mode = [None,None]
    mode[0] = x >= xy[0]
    mode[1] = y >= xy[1]        
    
    # mode[0] : 自機より左側 -> 中継点は右側
    if mode[0]:
        ref_c = col[1]
    else:
        ref_c = col[0]

    # mode[1] : 自機より上側 -> 中継点は下側
    ref_r = row[0]
    if len(row) > 1 and not mode[1]:            
        ref_r = row[1]
    
    return ref_r,ref_c