#!/usr/bin/env python
# -*- coding: utf-8 -*-

# '''
# Version 0.1 まずは1Hzで直進/後退をランダムにし続ける
# Version 0.2 TFから現在の位置を取得、目的地に向けてON/OFF制御
# conda install numba
# conda install -c conda-forge quaternion
# '''
import roslib
# roslib.load_manifest('learning_tf')
import rospy
import random
import math
import numpy as np
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose,Quaternion
from tf2_msgs.msg import TFMessage
import time
# import quaternion

version = 0.1

def rad2deg(x):
    return x * 180.0 / math.pi
def deg2rad(x):
    return x * math.pi/180.0

class ControlBot():
    def __init__(self, bot_name="NoName",mode="",*,type = 0,goal=[0.0,0.0,0.0]):        
        self.mode = mode
        self.gain_dist = np.zeros(3)
        self.gain_dir = np.zeros(3)
        self.gain_dir2 = np.zeros(3)
        
        # bot name 
        self.name = bot_name
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        # clock subscriber
        self.clk_sub = rospy.Subscriber('/clock',Clock,self.clk_callback)
        self.clk = 0.0  # current_time
        # tf subscriber 
        self.tf_sub = rospy.Subscriber("/tf", TFMessage, self.tf_callback)

        # 目標値 (位置,姿勢)
        rand = random.random()
        if mode == "dist":
            if rand < 0.5:                
                y = 1.0
            else:
                y = -1.0

            x = 0.0
            g_th = 0.0
        elif mode == "dir":
            if rand < 0.5:
                x = 1.0
            else:
                x = -1.0
            y = 0.0
            g_th = 0.0
        elif mode == "dir2":            
            if type == 0:
                g_th = - math.pi + deg2rad(1.0)
            elif type == 1:
                g_th = - 0.75 * math.pi
            elif type == 2:
                g_th = - 0.5 * math.pi
            elif type == 3:
                g_th = - 0.25*math.pi
            elif type == 4:
                g_th = 0.25 * math.pi
            elif type == 5:
                g_th = 0.5 * math.pi
            elif type == 6:
                g_th = 0.75 * math.pi
            else:
                g_th = math.pi - deg2rad(1.0)            
            x = 0.0
            y = 0.0
        elif mode == "random":
            x = 0.55*(random.random()-0.5)
            y = 2.0*(0.25-abs(x))*(random.random()  - 0.5)                
            g_th = 2.0*math.pi*(random.random()-0.5)
        elif mode == "goal":
            x = goal[0]
            y = goal[1]
            g_th = goal[2]

        self.goal = np.array([x,y])        
        self.g_th = g_th
        
        # 初期位置観測済みかどうか
        self.initialized = False

        # 目標値とのズレ
        self.e_dist = np.zeros(3)
        self.e_dir = np.zeros(3)
        self.e_dir2 = np.zeros(3)

        # 許容誤差
        self.threshold = 0.1        
        self.reward = 0             # 報酬
        self.flg = False
        self.cnt = 0
        self.isSucceeded = 0    # 状態                
        
        self.base_time2 = time.time()

    def calcTwist(self,trans,rot):                
        if not self.initialized:            
            self.init = np.array([trans[0],trans[1]])           
            # 初期状態での目標位置との距離
            self.init_dist = np.linalg.norm(self.goal - self.init)                                    
            self.initialized = True
            self.base_time = self.clk
            self.reward = self.timeout
        
        # 現在の位置と目標位置の距離        
        cur_pos = np.array([trans[0],trans[1]])        
        ref_vec = self.goal-cur_pos
        dist = np.linalg.norm(ref_vec)                

        # 現在の姿勢                
        cur_q = np.array([0,0,rot[2],rot[3]])
        cur_q = cur_q / np.linalg.norm(cur_q)
        
        # 現在の進行方向
        cur_dir = np.array([-cur_q[2]**2+cur_q[3]**2,2*cur_q[2]*cur_q[3]])
        cur_dir = cur_dir /np.linalg.norm(cur_dir)        

        theta = 0
        theta2 = 0
        if abs(dist) > self.threshold:
            # [目的地より遠い]
            self.e_dir2 = np.zeros(3)
            # 向かうべき方向ベクトル
            ref_dir = ref_vec / np.linalg.norm(ref_vec)

            # 方向ベクトルの内積＝向かうべき方向のとの角度誤差
            theta = math.atan2(np.cross(cur_dir,ref_dir),np.dot(cur_dir,ref_dir))
            
            # 角度誤差が大きい時は、バックで進行させる
            if theta < - math.pi/2 or math.pi/2 < theta:
                dist = -dist        
                theta = math.atan2(np.cross(-cur_dir,ref_dir),np.dot(-cur_dir,ref_dir))            
            
            self.e_dir[2] = self.e_dir[0]
            self.e_dir[0] = theta
            self.e_dir[1] += theta

        else:
            # [目的地付近]　回転を変更
            self.e_dir = np.zeros(3)

            # 目的の姿勢        
            ref_dir = np.array([math.cos(self.g_th),math.sin(self.g_th)])
            
            # 方向ベクトルの内積＝目標の向きとの角度誤差
            theta2 = math.atan2(np.cross(cur_dir,ref_dir),np.dot(cur_dir,ref_dir))            

            self.e_dir2[2] = self.e_dir2[0]
            self.e_dir2[0] = theta2
            self.e_dir2[1] += theta2
    
        self.e_dist[2] = self.e_dist[0]
        self.e_dist[0] = dist
        self.e_dist[1] += dist        

        
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.0
        # 距離・姿勢が目標値に近い時に報酬を与える
        isNearDist = abs(self.e_dist[0]) <= self.threshold and abs(self.e_dist[2]) <= self.threshold
        isNearDir = abs(self.e_dir2[0]) <= deg2rad(5.0) and abs(self.e_dir2[2]) <= deg2rad(5.0)

        if isNearDir and isNearDir:
            return twist
        if not isNearDist:
            twist.linear.x = np.dot(self.gain_dist,self.e_dist)             
            twist.angular.z = np.dot(self.gain_dir,self.e_dir)
        else:
            twist.linear.x = np.dot(self.gain_dist,self.e_dist)             
            twist.angular.z = np.dot(self.gain_dir2,self.e_dir2)

        return twist
      
    def tf_callback(self, tf_topic):        
        if tf_topic.transforms[0].child_frame_id == "base_footprint":
            self.trans = [
                tf_topic.transforms[0].transform.translation.x,
                tf_topic.transforms[0].transform.translation.y,
                tf_topic.transforms[0].transform.translation.z
            ]
            self.rot = [
                tf_topic.transforms[0].transform.rotation.x,
                tf_topic.transforms[0].transform.rotation.y,
                tf_topic.transforms[0].transform.rotation.z,
                tf_topic.transforms[0].transform.rotation.w
            ]

    def strategy(self,timeout=20.0):
        r = rospy.Rate(10) # change speed 1fps        
        self.trans = None
        self.rot = None
        self.timeout = timeout
        self.reward = timeout
        while not rospy.is_shutdown():
            try:                     
                if self.trans is not None and self.rot is not None:
                    twist = self.calcTwist(self.trans,self.rot)                       
                    self.vel_pub.publish(twist)
                    
                    # 報酬/状態の取得
                    self.calc_reward(self.trans,self.rot)                                
                if self.mode != "" and self.isSucceeded != 0:
                    return self.reward
                    
            except:
                import traceback
                traceback.print_exc()
                self.isSucceeded = -1                                        
            r.sleep()
        return self.reward

    def calc_reward(self,trans,rot):                                
        if not self.initialized:
            return
        
        # 時刻
        t = self.clk - self.base_time        
        
        # 距離・姿勢が目標値に近い時に報酬を与える
        isNearDist = abs(self.e_dist[0]) <= self.threshold and abs(self.e_dist[2]) <= self.threshold
        isNearDir = abs(self.e_dir2[0]) <= deg2rad(5.0) and abs(self.e_dir2[2]) <= deg2rad(5.0)

        
        if  isNearDist and isNearDir:
            if self.flg:
                self.cnt += 1
            else:
                self.flg = True

            if self.cnt > 30:
                self.isSucceeded = 1
        else:
            self.flg = False
            self.cnt = 0
        
        if t > self.timeout:
            self.isSucceeded = -1
            
        self.reward = t

    def clk_callback(self,clk_msg):        
        self.clk = clk_msg.clock.secs + clk_msg.clock.nsecs * 10e-9

    def set_goal(self,x,y,th):
        self.mode = ""
        self.e_dist = np.zeros(3)
        self.e_dir = np.zeros(3)
        self.e_dir2 = np.zeros(3)

        self.goal = np.array([x,y])
        self.g_th = th

        rospy.loginfo("set goal : [{} {} {}]".format(x,y,th))

    def wait(self,time):
        base_time = self.clk
        while self.clk-base_time < time:
            time.sleep(0.05)
        return
    
    def wait_goal(self,timeout=20):
        base_time = self.clk
        while self.clk-base_time < timeout:
            if self.isSucceeded == 1:
                return
            time.sleep(0.05)
        
if __name__ == '__main__':
    rospy.init_node('control_node')
    
    bot = ControlBot(version)
    bot.strategy()

