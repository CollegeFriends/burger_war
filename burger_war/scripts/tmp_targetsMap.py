import math

import rospy
from geometry_msgs.msg import PoseStamped
from burger_war.msg import war_state
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def getTargetsMapOnGAZEBO():
    Targets = {
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
    return Targets


def getTargetsMap():
    Targets = {
        "FriedShrimp_N":    [+0.35/2.0,         0.0],
        "FriedShrimp_S":    [-0.35/2.0,         0.0],
        "FriedShrimp_E":    [0.0,               -0.35/2.0],
        "FriedShrimp_W":    [0.0,               +0.35/2.0],
        "Omelette_N":       [+0.53+0.15/2.0,    -0.53],
        "Omelette_S":       [+0.53-0.15/2.0,    -0.53],
        "Tomato_N":         [+0.53+0.15/2.0,    +0.53],
        "Tomato_S":         [+0.53-0.15/2.0,    +0.53],
        "OctopusWiener_N":  [-0.53+0.15/2.0,    -0.53],
        "OctopusWiener_S":  [-0.53-0.15/2.0,    -0.53],
        "Pudding_N":        [-0.53+0.15/2.0,    +0.53],
        "Pudding_S":        [-0.53-0.15/2.0,    +0.53],
    }
    return Targets


distance = 0.3  # magic parm of distance to the target


def getGoal(targetPos, targetName):
    goalPos = targetPos
    goalDire = [0.0, 0.0, 0.0, 1.0]

    if targetName[-1] == "N":
        goalPos[0] = goalPos[0] + distance
        goalDire[2] = 1.0
        goalDire[3] = 0.0
    elif targetName[-1] == "S":
        goalPos[0] = goalPos[0] - distance
        goalDire[2] = 0.0
        goalDire[3] = 1.0
    elif targetName[-1] == "W":
        goalPos[1] = goalPos[1] + distance
        goalDire[2] = math.sin(-math.pi/4.0)
        goalDire[3] = math.cos(-math.pi/4.0)
    elif targetName[-1] == "E":
        goalPos[1] = goalPos[1] - distance
        goalDire[2] = math.sin(math.pi/4.0)
        goalDire[3] = math.cos(math.pi/4.0)
    elif targetName == "enemy":
        print("Target is ENEMY!!")
    else:
        print("ERROR!!!")

    target_pos = PoseStamped()
    target_pos.header.frame_id = "map"
    target_pos.pose.position.x = goalPos[0]
    target_pos.pose.position.y = goalPos[1]
    target_pos.pose.orientation.z = goalDire[2]
    target_pos.pose.orientation.w = goalDire[3]

    return target_pos


def getGoal2(targetPos, targetName):
    goalPos = targetPos
    goalDire = [0.0, 0.0, 0.0, 1.0]

    if targetName[-1] == "N":
        goalPos[0] = goalPos[0] + distance
        goalDire[2] = 1.0
        goalDire[3] = 0.0
    elif targetName[-1] == "S":
        goalPos[0] = goalPos[0] - distance
        goalDire[2] = 0.0
        goalDire[3] = 1.0
    elif targetName[-1] == "W":
        goalPos[1] = goalPos[1] + distance
        goalDire[2] = math.sin(-math.pi/4.0)
        goalDire[3] = math.cos(-math.pi/4.0)
    elif targetName[-1] == "E":
        goalPos[1] = goalPos[1] - distance
        goalDire[2] = math.sin(math.pi/4.0)
        goalDire[3] = math.cos(math.pi/4.0)
    elif targetName == "enemy":
        print("Target is ENEMY!!")
    else:
        print("ERROR!!!")

    target_pos = MoveBaseGoal()
    target_pos.header.frame_id = "map"
    target_pos.pose.position.x = goalPos[0]
    target_pos.pose.position.y = goalPos[1]
    target_pos.pose.orientation.z = goalDire[2]
    target_pos.pose.orientation.w = goalDire[3]

    return target_pos


def getNearestTarget(target_map, x, y, war_state):
    war_state_dict = converter(war_state)

    dist = 99.0
    nearestTarget = ""
    for key in target_map:
        tmp = (target_map[key][0] - x) ** 2 + (target_map[key][1] - y) ** 2
        if war_state_dict[key]['owner'] != war_state.my_side and tmp < dist:
            nearestTarget = key
            dist = tmp

    return nearestTarget


def isGetTarget(target_name, war_state):
    war_state_dict = converter(war_state)
    if war_state_dict[target_name]['owner'] != war_state.my_side:
        print("Apple pen ")
        return True
    else:
        return False


def converter(war_state):
    war_state_dict = {}

    for i in range(len(war_state.target_names)):
        war_state_i = {}
        war_state_i['owner'] = war_state.target_owner[i]
        war_state_i['point'] = war_state.target_point[i]
        war_state_dict[war_state.target_names[i]] = war_state_i
    return war_state_dict
