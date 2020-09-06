#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import tf
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import target_map
from copy import deepcopy as dcopy
from geometry_msgs.msg import PoseStamped
from burger_controller.msg import war_state
from actionlib_msgs.msg import GoalStatusArray
from burger_controller.msg import CvRect
from geometry_msgs.msg import Twist
from jsk_rviz_plugins.msg import OverlayText
from sensor_msgs.msg import LaserScan
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import time
import numpy as np
from collections import deque

class MyStateBot(object):

    def __init__(self, mySide='r'):
        self.mySide = mySide
        self.map = target_map.getTargetsMap()

        self.goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=5)
        self.my_state_pub = rospy.Publisher('my_state', OverlayText, queue_size=5)
        self.limited_cmd_vel_sub = rospy.Subscriber('limited_cmd_vel', Twist, self.limitedCmdVelCallback)
        self.war_state_sub = rospy.Subscriber('war_state', war_state, self.warStateCallback)
        self.nav_state_sub = rospy.Subscriber('move_base/status', GoalStatusArray, self.navStateCallback)        
        self.cv_rect_sub = rospy.Subscriber('cv_rect', CvRect, self.cvRectSubCallback)
        self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1000)
        self.laser_sub = rospy.Subscriber('scan', LaserScan, self.laserCallback)
        
        self.limitedCmdVel = Twist()
        self.war_state = war_state()
        self.war_state.target_names = []
        self.war_state.target_owner = []
        self.war_state.target_point = []
        self.range_data = 0.0
        self.laser_data = None
        self.my_state_text = OverlayText()
        self.my_state_text.text = ""

        self.r = 10.0   # rospy frequency

        self.enemy_found_base_time = rospy.Time.now()
        self.enemy_found_mode = 0
        """
        0   : 敵検知していない
        1   : 赤印を検知
        2   : 緑印を検知
        -1  : 不感時間 (5秒後に0に自動遷移)        
        """
        

        self.isFoundEnemy = False
        self.isFoundEnemyMarker = False
        self.rectData = CvRect()
        self.navStatus = ""

        # Parameters of PID controller
        self.Kp = 0.005
        self.Ki = 0.0
        self.Kd = 0.00005
        self.integral = 0.0
        self.error = 0.0
        self.error_pre = 0.0
        self.dt = 1.0/self.r
        self.target_001 = 320.0

        # Parameters of DIST
        self.threshold_r = 50.0  # threshold of red area width
        self.threshold_g = 30.0  # threshold of green area width

    def strategy(self):

        r = rospy.Rate(self.r)
        nearestTargetName = "ROBOT"
        nearestTargetName_pre = nearestTargetName
        tf_listener = tf.TransformListener()                
        base_time = rospy.Time.now()

        goal_que = deque([])
        mode_flg = False
        while not rospy.is_shutdown():
            try:
                target_pos = PoseStamped()

                # 自機の位置を取得
                (trans, rot) = tf_listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
                self.pose_x = trans[0]
                self.pose_y = trans[1]                
                
                # 最も近いターゲットの名前を取得                
                nearestTargetName = target_map.getNearestTarget(dcopy(self.map), dcopy(self.pose_x), dcopy(self.pose_y), self.war_state)
                
                # 敵検知
                self.searchEnemy()

                # 制御
                self.pubTwistWithPIDController()

                self.my_state_text.text  = "My side is : " + str(self.mySide) + "\n"                
                if self.isFoundEnemyMarker:
                    nearestTargetName = "ROBOT"
                    self.my_state_text.text += "Current Target : ROBOT (GREEN)"
                elif self.isFoundEnemy:
                    nearestTargetName = "ROBOT"
                    self.my_state_text.text += "Current Target : ROBOT (RED)"
                elif nearestTargetName == "":
                    self.my_state_text.text += "All Targets are MINE !!!!!"
                else:
                    self.my_state_text.text += "Current Target : " + nearestTargetName
                    nearestTargetPos = dcopy(self.map[nearestTargetName])
                    target_pos = target_map.getGoal(nearestTargetPos, nearestTargetName)
                    target_pos.header.stamp = rospy.Time.now()

                self.my_state_text.text += "\nRect R Area :"  + str(self.rectData.rect_r.area) \
                                        + "\t Width :" + str(self.rectData.rect_r.length[0]) \
                                        + "\t Height :" + str(self.rectData.rect_r.length[1])
                self.my_state_text.text += "\nRect G Area :" + str(self.rectData.rect_g.area) \
                                        + "\t Width :" + str(self.rectData.rect_g.length[0]) \
                                        + "\t Height :" + str(self.rectData.rect_g.length[1])
                self.my_state_text.text += "\nRect B Area :" + str(self.rectData.rect_b.area) \
                                        + "\t Width :" + str(self.rectData.rect_b.length[0]) \
                                        + "\t Height :" + str(self.rectData.rect_b.length[1])

                self.my_state_text.text += "\nEnemy found mode:" + str(self.enemy_found_mode)
                if self.enemy_found_mode == 0:
                    self.my_state_text.text += "\t None"
                elif self.enemy_found_mode == 1:
                    self.my_state_text.text += "\t Red"
                elif self.enemy_found_mode == 2:
                    self.my_state_text.text += "\t Green"
                elif self.enemy_found_mode == -1:
                    self.my_state_text.text += "\t -"
                    
                    
                self.my_state_text.text +="\nRange Data [180]:" + str(self.range_data)
                self.my_state_text.text +="\nNav Status:" + self.navStatus                
                self.my_state_pub.publish(self.my_state_text)

                # 目標の更新
                if nearestTargetName != nearestTargetName_pre:
                    nearestTargetName_pre = nearestTargetName
                    
                    self.goal_pub.publish(target_pos)

                    if nearestTargetName != "ROBOT" and nearestTargetName != "":
                        goal_que.append(target_pos)                    
                    
                    base_time = rospy.Time.now()

                if rospy.Time.now() - base_time > rospy.Duration(10):                    
                    # 10秒経過しても到着しなかった時は、前回の目標座標を再送
                    self.escape()
                    self.goal_pub.publish(goal_que.popleft())                    
                    base_time = rospy.Time.now()

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr("Failed to get transform")
            finally:                
                r.sleep()

    def warStateCallback(self, data):
        self.war_state = data

    def navStateCallback(self, data):
        if len(data.status_list) > 0:
            # 状態が取得できた場合
            status = data.status_list[0]
            self.navStatus = str(status.status)
            # rospy.logerr(status.status)
            # if status.status == 3:
            #     # self.isReachedGoalMode = True

    def cvRectSubCallback(self, data):
        self.rectData = data        

    def limitedCmdVelCallback(self,data):
        self.limitedCmdVel = data

    def searchEnemy(self):
        try:        
            self.enemy_r = self.rectData.rect_r
            self.enemy_g = self.rectData.rect_g
            now = rospy.Time.now()

            if self.enemy_found_mode == 0:
                # 敵を検知していなかった時は、下記条件で敵を発見            
                if self.enemy_g.center != (-1.0, -1.0) and self.enemy_g.length[1] > 100.0:
                    # 緑枠を検知
                    self.isFoundEnemyMarker = True
                    self.isFoundEnemy = True
                    self.enemy_found_mode = 2
                    self.enemy_found_base_time = now
                elif self.enemy_r.center != (-1.0, -1.0) and self.enemy_r.length[0] > 60.0:
                    # 赤枠を検知
                    self.isFoundEnemy = True
                    self.enemy_found_mode = 1
                    self.enemy_found_base_time = now
            elif self.enemy_found_mode > 0:
                if now - self.enemy_found_base_time > rospy.Duration(2):
                    # 検知開始から1秒の間は、必ず逃げる
                    return
                if now - self.enemy_found_base_time > rospy.Duration(5):
                    # 5秒以上経過したら、敵検知を一時中断
                    self.isFoundEnemyMarker = False
                    self.isFoundEnemy = False
                    self.enemy_found_mode = -1
                    self.enemy_found_base_time = now
                    return

                # 敵を検知したときは、下記条件で遠ざかったことを判定        
                if self.enemy_g.center != (-1.0, -1.0) or self.enemy_g.length[1] < 10.0:
                    # 緑枠を検知しなくなった
                    self.isFoundEnemyMarker = False
                    self.isFoundEnemy = False
                    self.enemy_found_mode = -1
                    self.enemy_found_base_time = rospy.Time.now()                
                elif self.enemy_r.center != (-1.0, -1.0) or self.enemy_r.length[0] < 10.0:                
                    # 赤枠を検知しなくなった
                    self.isFoundEnemyMarker = False
                    self.isFoundEnemy = False
                    self.enemy_found_mode = -1
                    self.enemy_found_base_time = rospy.Time.now()           
            else:                
                if now -self.enemy_found_base_time > rospy.Duration(1):
                    # 1秒経過で敵未検知の状態に変更
                    self.enemy_found_mode = 0
                    self.enemy_found_base_time = now
        except:
            import traceback
            traceback.print_exc()

    def pubTwistWithPIDController(self):        

        twist = Twist()        
        if self.isFoundEnemyMarker:
            # 敵機の緑枠を見つけた時
            self.error = self.target_001 - self.enemy_g.center[0]
            self.integral += self.error * self.dt
            twist.angular.z = self.Kp * self.error + self.Kd * (self.error - self.error_pre) / self.dt            
            if self.range_data > 0.1:
                twist.linear.x = -0.22
            self.twist_pub.publish(twist)
        elif self.isFoundEnemy:
            # 敵機の赤印を見つけた時
            self.error = self.target_001 - self.enemy_r.center[0]
            self.integral += self.error * self.dt
            twist.angular.z = self.Kp * self.error + self.Kd * (self.error - self.error_pre) / self.dt            
            if self.range_data > 0.1:
                twist.linear.x = -0.22
            self.twist_pub.publish(twist)            
        else:
            self.error = 0.0
            self.error_pre = 0.0
            self.integral = 0.0
            self.angular_z_PID = 0.0
        

    def laserCallback(self, data):        
        laser_np = np.array([data.ranges[180-22:180+22]])        
        self.laser_data = np.array([data.ranges])        
        self.range_data = laser_np.min()

    def escape(self):
        base_time = rospy.Time.now()
        base_twist = self.limitedCmdVel
        
        while rospy.Time.now() - base_time < rospy.Duration(5):
            twist = Twist()
            if  base_twist.linear.x > 0:
                twist.linear.x = -0.22                        
            elif base_twist.linear.x < 0:
                twist.linear.x = 0.22                
            self.twist_pub.publish(twist)

            if base_twist.linear.x > 0 and self.laser_data[0:22] > 0.1 and self.laser_data[360-22:] > 0.1:
                break
            if base_twist.linear.x > 0 and self.laser_data[1800-22:180+22] > 0.1:
                break

            

            
    
if __name__ == '__main__':
    rospy.init_node('my_state')
    mySide = rospy.get_param("side")
    rospy.loginfo("MY SIDE IS {}".format(mySide))    
    bot = MyStateBot(mySide=mySide)
    bot.strategy()
