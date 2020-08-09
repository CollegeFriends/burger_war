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
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose,Quaternion
import tf
import time
# import quaternion

version = 0.1
def rad2deg(x):
    return x * 180.0 / math.pi

class ControlBot():
    def __init__(self, bot_name="NoName"):
        # bot name 
        self.name = bot_name
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)

        # 目標値 (位置,姿勢)
        x = 0.55*(random.random()-0.5)
        y = 2.0*(0.25-abs(x))*(random.random()  - 0.5)
        self.goal = np.array([x,y])
        self.g_th = 2.0*math.pi*(random.random()-0.5)                
        
        # 初期位置観測済みかどうか
        self.initialized = False

        # 許容誤差
        self.threshold = 0.01
        
        self.pre_dist = None    # 直前の距離
        self.reward = 0         # 報酬
        self.isSucceeded = 0    # 状態                

        self.strategy()
        
    def calcTwist(self,trans,rot):                
        if not self.initialized:            
            self.init = np.array([trans[0],trans[1]])           
            # 初期状態での目標位置との距離
            self.init_dist = np.linalg.norm(self.goal - self.init)                                    
            self.initialized = True
        
        if self.isSucceeded == 1:
            return

        # 現在の位置と目標位置の距離
        # dist : 初期の距離からの進行度合い
        cur_pos = np.array([trans[0],trans[1]])        
        ref_vec = self.goal-cur_pos
        dist = np.linalg.norm(ref_vec)        
        
        # 向かうべき方向ベクトル
        ref_dir = ref_vec / np.linalg.norm(ref_vec)

        # 現在の姿勢                
        cur_q = np.array([0,0,rot[2],rot[3]])
        cur_q = cur_q / np.linalg.norm(cur_q)
        
        # 現在の進行方向
        cur_dir = np.array([-cur_q[2]**2+cur_q[3]**2,2*cur_q[2]*cur_q[3]])
        cur_dir = cur_dir /np.linalg.norm(cur_dir)
        # rospy.logdebug("dir : {}".format(cur_dir))

        # 方向ベクトルの内積＝向かうべき方向のとの角度誤差
        theta = math.atan2(np.cross(cur_dir,ref_dir),np.dot(cur_dir,ref_dir))

        x = 0.0
        th = 0.0

        
        if dist >= self.threshold:                                    
            x = dist
            if abs(theta)> 0.2 * math.pi / 180.0:
                th = 1.0 * theta            
            else:            
                th = 0.0
            
        else:
            x = 0.0
            # 目標の姿勢角
            ref_dir = np.array([math.cos(self.g_th),math.sin(self.g_th)])
            # 角度誤差
            theta = math.atan2(np.cross(cur_dir,ref_dir),np.dot(cur_dir,ref_dir))
            if theta > math.pi:
                theta = theta -2.0*math.pi
            elif theta < - math.pi:
                theta = theta + 2.0 * math.pi

            if abs(theta) > 0.2 * math.pi / 180.0:
                th = 0.5 * theta            

            else:            
                th = 0.0            


        
        twist = Twist()
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th

        return twist
        

    def strategy(self):
        r = rospy.Rate(1) # change speed 1fps
        
        listener = tf.TransformListener()
        self.base_time = time.time()
        
        while not rospy.is_shutdown() and self.isSucceeded == 0:
            try:
                (trans,rot)= listener.lookupTransform('odom','base_footprint',rospy.Time(0))                                
                twist = self.calcTwist(trans,rot)    
                self.vel_pub.publish(twist)

                # 報酬/状態の取得
                self.calc_reward(trans,rot)                
                # rospy.loginfo(self.reward)

            except:
                # rospy.logwarn("TF 読み取り ERROR")
                import traceback
                traceback.print_exc()
                pass
                        
            r.sleep()

        return self.reward

    def calc_reward(self,trans,rot):                                
        if not self.initialized:
            return

        # dist : 初期の距離からの進行度合い
        cur_pos = np.array([trans[0],trans[1]])        
        ref_vec = self.goal-cur_pos
        dist = np.linalg.norm(ref_vec)        
        if self.pre_dist is None:
            self.pre_dist = dist
        
        # 距離による報酬        
        reward = (self.pre_dist-dist) / self.pre_dist

        # 現在の姿勢                
        cur_q = np.array([0,0,rot[2],rot[3]])
        cur_q = cur_q / np.linalg.norm(cur_q)
        
        # 現在の進行方向        
        cur_dir = np.array([-cur_q[2]**2+cur_q[3]**2,2*cur_q[2]*cur_q[3]])
        cur_dir = cur_dir /np.linalg.norm(cur_dir)
        # 目標の姿勢角
        ref_dir = np.array([math.cos(self.g_th),math.sin(self.g_th)])
        
        # 角度誤差
        theta = math.atan2(np.cross(cur_dir,ref_dir),np.dot(cur_dir,ref_dir))
        print(rad2deg(theta))
        if dist < self.threshold and abs(theta) < 10*math.pi/180.0:
            self.isSucceeded = 1

        # 時刻による罰則
        t = time.time()- self.base_time
        if t > 10:
            self.isSucceeded = -1        
            rospy.logwarn("time out : 10 s")

            twist = Twist()
            twist.linear.x = 0;twist.linear.y = 0;twist.linear.z = 0
            twist.angular.x = 0;twist.angular.y = 0;twist.angular.z = 0
            self.vel_pub.publish(twist)
        
        # 報酬の決定
        self.reward += reward * math.exp(-t/3.0)
        if t > 3 :
            self.reward -= 0.005 * (1.0 - math.exp(-t/3.0))
    
if __name__ == '__main__':
    rospy.init_node('control_node')
    
    bot = ControlBot(version)
    bot.strategy()

