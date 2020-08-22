#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Ver
# 0.0 : Laserのデータを取得する
# 0.1 : 現在位置のデータを取得する
# 1.0 : 障害物の距離が 0.20 (= 20cm) 未満となったら終了する (※robotのサイズが138x178x192mm)
# 1.1 : リセットをかけてから実行する
# 1.2 : Sim上の時間を使って10minになったら終了する
# 2.0 : ランダムな走行の実装
# 2.1 : 終了時はロボットが完全停止するように変更
# 2.2 : timeout = Noneで無限に走行を可能に変更
# 2.3 : 引数で敵機体の制御可能に変更


import math
import random
import numpy as np
import rospy
import tf
from sensor_msgs.msg import Image, LaserScan
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist

def Rad2Deg(rad):
    return rad * 180.0 / math.pi
def Deg2Rad(deg):
    return deg * math.pi / 180.0

class runBot():
    def __init__(self,timeout = None, ns = None):        
        if ns is None:
            self.group_name = "/"
        else:
            self.group_name = "/"+ ns + "/"

        self.tf_sub = rospy.Subscriber("/tf", TFMessage, self.tf_callback)        
        self.clk_sub = rospy.Subscriber('/clock',Clock,self.clk_callback)
        self.scan_sub = rospy.Subscriber(self.group_name+"scan", LaserScan, self.scan_callback)        
        self.vel_pub = rospy.Publisher(self.group_name+'cmd_vel', Twist,queue_size=1)

        self.clk = 0.0          # 現在時刻
        self.base_time = None   # 開始時刻
        self.timeout = timeout  # 終了時刻

        self.cur_pos = None # 現在位置
        self.cur_dir = None # 現在の向き(x軸と進行方向とのなす角度)



        self.end_flg = False    # 終了フラグ
        self.succseeded = 0     # 成功したかどうか (-1:接近し過ぎ,-2:タイムアウト)
        
    def scan_callback(self, scan):
        self.scan = np.array(scan.ranges)         
        if np.min(self.scan) < 0.2:
            self.succseeded = -1
            self.end_flg = True    
    
    def tf_callback(self, tf_topic):        
        if tf_topic.transforms[0].child_frame_id == "base_footprint":
            topic = tf_topic.transforms[0]
            self.cur_pos = np.array([topic.transform.translation.x, topic.transform.translation.y])

            rot = [topic.transform.rotation.x,topic.transform.rotation.y,topic.transform.rotation.z,topic.transform.rotation.w]
            self.cur_dir = tf.transformations.euler_from_quaternion(rot)[2]
            # print(self.cur_pos,Rad2Deg(self.cur_dir))
    
    def clk_callback(self,clk_msg):        
        self.clk = clk_msg.clock.secs + clk_msg.clock.nsecs * 10e-9
        if self.base_time is None:
            self.base_time = self.clk
        
        if self.timeout is None:
            return
        elif self.clk - self.base_time > self.timeout:
            self.succseeded = -2
            self.end_flg = True

    def calcTwist(self):
        value = random.randint(1,1000)
        if value < 250:
            x = 0.2
            th = 0
        elif value < 500:
            x = -0.2
            th = 0
        elif value < 750:
            x = 0
            th = 1
        elif value < 1000:
            x = 0
            th = -1
        else:
            x = 0
            th = 0
        twist = Twist()
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        return twist

    def strategy(self):   
        r = rospy.Rate(100) # 100Hzで終了判定        
        while not rospy.is_shutdown():
            self.vel_pub.publish(self.calcTwist())

            if self.end_flg:
                break
            r.sleep()            
        return self.succseeded

if __name__ == "__main__":
    from testUtility import *
    reset_sim()
    rospy.init_node("find_wall")    
    
    side = "RED"
    
    if side == "RED":
        ns = None
    elif side == "BLUE":
        ns = "enemy_bot"
        
    bot = runBot(ns=ns)
    score = bot.strategy()    
    if score == -1:
        rospy.loginfo("{} >> 障害物に近すぎた".format(side))
    elif score == -2:
        rospy.loginfo("{} >> Timeout".format(side))
    bot.vel_pub.publish(Twist())





