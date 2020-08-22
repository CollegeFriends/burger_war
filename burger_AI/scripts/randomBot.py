#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Ver
# 0.0 : Laserのデータを取得する
# 0.1 : 現在位置のデータを取得する
# 1.0 : 障害物の距離が 0.20 (= 20cm) 未満となったら終了する (※robotのサイズが138x178x192mm)
# 1.1 : リセットをかけてから実行する
# 1.2 : Sim上の時間を使って10minになったら終了する

import math
import rospy
import numpy as np
import tf
from sensor_msgs.msg import Image, LaserScan
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
import matplotlib.pyplot as plt

def Rad2Deg(rad):
    return rad * 180.0 / math.pi
def Deg2Rad(deg):
    return deg * math.pi / 180.0

class runBot():
    def __init__(self,timeout = 10.0):
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)        
        self.tf_sub = rospy.Subscriber("/tf", TFMessage, self.tf_callback)        
        self.clk_sub = rospy.Subscriber('/clock',Clock,self.clk_callback)
        
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
        
        if self.clk - self.base_time > self.timeout:
            self.succseeded = -2
            self.end_flg = True
   
    def strategy(self):   
        r = rospy.Rate(100) # 100Hzで終了判定        
        while not rospy.is_shutdown():
            plt.cla()
            if self.end_flg:
                break
            r.sleep()            
        return self.succseeded

if __name__ == "__main__":
    from testUtility import *
    reset_sim()
    rospy.init_node("find_wall")    
    bot = runBot()    
    score = bot.strategy()
    if score == -1:
        print("近くにいすぎた")
    elif score == -2:
        print("タイムアウト")





