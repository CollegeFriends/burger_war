#!/usr/bin/env python
# -*- coding: utf-8 -*-

# '''
# Version 0.1 まずは1Hzで直進/後退をランダムにし続ける
# Version 0.2 TFから現在の位置を取得、目的地に向けてON/OFF制御
# Version 2.0 目標値のスタック
# Version 2.1 移動失敗時に一つ前の目的地に戻り、リトライ可能に
# Version 3.0 AMCLから現在位置を取得するように変更
# Version 3.1 war_stateの受信
# '''

import roslib
# roslib.load_manifest('learning_tf')
import rospy
import tf
import random
import math
import numpy as np
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose,Quaternion
from burger_war.msg import war_state
from tf2_msgs.msg import TFMessage
import time
from targetMapUtil import *

version = 3.1

def rad2deg(x):
    return x * 180.0 / math.pi
def deg2rad(x):
    return x * math.pi/180.0

class ControlBot():
    def __init__(self, bot_name="NoName"):                        
        # ゲイン
        Kp_dist = 0.9731625194682382
        Ki_dist = 0.00016981785117853974    
        Kp_dir = 4.524248499803513
        Ki_dir = 0.0002950570968987406
        Kp_dir2 = 3.2620494609668675
        Ki_dir2 = 0.027002882654888146
        self.gain_dist = np.array([Kp_dist,Ki_dist,0.0])
        self.gain_dir = np.array([Kp_dir,Ki_dir,0.0])
        self.gain_dir2 = np.array([Kp_dir2,Ki_dir2,0.0])
        
        # bot name 
        self.name = bot_name
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        # war state subscriber
        self.warState = None       
        self.war_state_sub = rospy.Subscriber('/war_state', war_state, self.warStateCallback)

        # clock subscriber
        self.clk_sub = rospy.Subscriber('/clock',Clock,self.clk_callback)
        self.clk = 0.0  # current_time
        

        # 現在位置・姿勢
        self.trans = None
        self.rot = None


        # 移動の状態
        self.init = None            # 移動開始位置
        self.init_dist = None       # 移動開始時の距離
        self.base_time = None       # 移動開始時の時刻
        self.initialized = False    # 初期化済み(移動開始済み)かどうか
        self.retry = False          # 失敗した時に一つ前の地点に戻り、リトライする

        # 目標値
        self._queue = []
        self._path  = []
        self.goal = np.zeros(2)
        self.g_th = None
        
        # 目標値とのズレ
        self.e_dist = np.zeros(3)
        self.e_dir = np.zeros(3)
        self.e_dir2 = np.zeros(3)

        # 許容誤差
        self.threshold = 0.075        # 位置の誤差                
        self.threshold2 = 5.0       # 角度の誤差
        self.flg = 0                # フラグ        
        self.isSucceeded = 1        # 状態                        

        # 時刻
        self.base_time2 = time.time()   # 移動開始時刻
        self.timeout = 20               # タイムアウト 

    def calcTwist(self):                
        if not self.initialized:            
            # 初期化処理            
            self.init = np.array([self.trans[0],self.trans[1]])     # 移動開始位置の設定                       
            self.init_dist = np.linalg.norm(self.goal - self.init)  # 移動開始時の距離
            self.base_time = self.clk                               # 移動開始時刻の設定
            self.isSucceeded = 0                                    # 移動完了状態のクリア (0: 移動中 / 1: 移動完了(待機中) / -1: タイムアウト(移動失敗))
            self.flg = 0                                            # 移動状況のクリア (0: 未達 / 1: 近接位置に来た)            
            self.e_dist = np.zeros(3)                               # 誤差の初期化
            self.e_dir = np.zeros(3)                                # 誤差の初期化
            self.e_dir2 = np.zeros(3)                               # 誤差の初期化
            self.initialized = True                                 # 初期化完了
            rospy.loginfo("Initialized :[ {} {} {} {} ]".format(self.goal[0],self.goal[1],self.g_th,self.base_time))
            
        if self.isSucceeded != 0 or self.goal is None:            
            # 移動完了or タイムアウト or 目的地がない場合
            twist = Twist()
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.0                    
            return twist        

        # 現在の位置と目標位置の距離        
        cur_pos = np.array([self.trans[0],self.trans[1]])        
        ref_vec = self.goal-cur_pos
        dist = np.linalg.norm(ref_vec)                

        # 現在の姿勢                
        cur_q = np.array([0,0,self.rot[2],self.rot[3]])
        cur_q = cur_q / np.linalg.norm(cur_q)
        
        # 現在の進行方向        
        cur_dir = np.array([-2*cur_q[2]*cur_q[3],-cur_q[2]**2+cur_q[3]**2])
        cur_dir = cur_dir /np.linalg.norm(cur_dir)

        theta = 0
        theta2 = 0

        if self.flg == 0 and abs(dist) > self.threshold:
            # [目的地より遠い = 移動]
            
            # 姿勢調整用変数をクリア
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
            # [目的地付近 = 姿勢を調整]

            # 移動方向調整用変数をクリア
            self.e_dir = np.zeros(3)

            # モードを変換 (近接位置に到達した)
            self.flg = 1

            if self.g_th is None:
                # 姿勢の指定がない場合はクリア
                pass
            else:
                # 目的の姿勢を算出        
                ref_dir = np.array([math.cos(self.g_th),math.sin(self.g_th)])
                
                # 方向ベクトルの内積＝目標の向きとの角度誤差
                theta2 = math.atan2(np.cross(cur_dir,ref_dir),np.dot(cur_dir,ref_dir))            

                self.e_dir2[2] = self.e_dir2[0]
                self.e_dir2[0] = theta2
                self.e_dir2[1] += theta2
        
        self.e_dist[2] = self.e_dist[0]
        self.e_dist[0] = dist
        self.e_dist[1] += dist               
        
        # 位置が目標値に近い時に状態を変更
        isNearDist = (abs(self.e_dist[0]) <= self.threshold) and (abs(self.e_dist[2]) <= self.threshold)
        # 姿勢の指定がないか、目標値に近い時に状態を変更
        isNearDir = (self.g_th is None) or (abs(self.e_dir2[0]) <= deg2rad(self.threshold2) and abs(self.e_dir2[2]) <= deg2rad(self.threshold2))
        
        # 制御信号の演算
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.0        
        
        if isNearDist and isNearDir:
            # 移動完了
            rospy.loginfo("Success : [ {:.03f} {:.03f} {:.03f} ]".format(cur_pos[0],cur_pos[1],self.e_dir2[0]))
            self.isSucceeded = 1
            # 停止信号送信(左右ともに空信号)            
            pass
        elif not isNearDist:
            # 移動
            twist.linear.x = np.dot(self.gain_dist,self.e_dist)             
            twist.angular.z = np.dot(self.gain_dir,self.e_dir)
        else:
            # 姿勢調整            
            twist.angular.z = np.dot(self.gain_dir2,self.e_dir2)

        return twist

    def clk_callback(self,clk_msg):        
        self.clk = clk_msg.clock.secs + clk_msg.clock.nsecs * 10e-9
   
    def warStateCallback(self, data):        
        self.warState = data
        


    def strategy(self,RETRY=False,END=False):
        # 制御周期 = 10 Hz
        r = rospy.Rate(10)
        self.trans = None
        self.rot = None        
        self.retry = RETRY
        self.nearest_target_name = None
        tf_listener = tf.TransformListener()

        while not rospy.is_shutdown():
            try:
                nearest_target_name = None
                # 現在地/近接ターゲットの取得                                                   
                try:
                    (trans,rot) = tf_listener.lookupTransform('map', 'base_footprint', rospy.Time(0))                
                    self.trans = np.array([-trans[1],trans[0],trans[2]])
                    self.rot = np.array([rot[0],rot[1],rot[2],rot[3]])
                    nearest_target_name = getNearestTarget(self.trans[0],self.trans[1],self.warState)                                        
                except:                    
                    import traceback
                    traceback.print_exc()
                    print("HEY")

                # 最も近いターゲットが変わった場合
                if nearest_target_name is not None:                    
                    if self.nearest_target_name is None or self.nearest_target_name != nearest_target_name:                                        
                        self.nearest_target_name = nearest_target_name
                        rospy.loginfo("nearest_target : {}".format(self.nearest_target_name))                                        
                        self.set_target(self.nearest_target_name)

                # 制御信号の計算 
                if self.trans is not None and self.rot is not None:
                    # 現在位置が不明でない時                            
                    self.update_goal()           # 移動先の指定
                    twist = self.calcTwist()     # 制御信号の演算
                    rospy.logdebug("{} {} {}".format(twist.linear.x,twist.linear.y,twist.angular.z))
                else:
                    # 現在位置が不明な時 = 静止
                    twist = Twist()
                    twist.linear.x = 0;     twist.linear.y = 0;     twist.linear.z =0                    
                    twist.angular.x = 0;    twist.angular.y = 0;    twist.angular.z = 0                

                # 制御信号の送信
                self.vel_pub.publish(twist)   

                if END and len(self._queue)<1 and self.goal is None:
                    return                  
            except:
                # import traceback
                # traceback.print_exc()
                self.isSucceeded = -1
            r.sleep()

    # 目的地の更新
    def update_goal(self):
        if self.isSucceeded == 1:    
            # 正常にゴールに到達した場合
            if len(self._queue) < 1:
                self._path.append([self.goal[0],self.goal[1],self.g_th,self.timeout])
                self.goal = None
                self.g_th = None
                self.isSucceeded = 1
                self.initialized = True                
                rospy.loginfo("Update goal : STOP [EMPTY]")
            else:                      
                self._path.append([self.goal[0],self.goal[1],self.g_th,self.timeout])          
                tmp = self._queue[0]
                self._queue.pop(0)                
                self.base_time = self.clk
                self.isSucceeded = 0
                self.goal = np.array([tmp[0],tmp[1]])
                self.g_th = tmp[2]                
                self.timeout = tmp[3]
                self.initialized = False            
                rospy.loginfo("Update goal : {} {} {} {}".format(tmp[0],tmp[1],tmp[2],tmp[3]))
        elif self.isSucceeded == -1:              
            if not self.retry:
                self.isSucceeded = 1
                return
            # ゴール到達に失敗した場合
            if len(self._path) < 1:
                #どこにも到達できていなかった場合、その場で静止
                self.stop()
            else:                
                # RETRYする場合
                rospy.loginfo("RETRY!")
                # 現在の目的地を目標リストの先頭に再追加
                self._queue.insert(0,[self.goal[0],self.goal[1],self.g_th,self.timeout])                
                #最後に到達した地点を目的地に設定
                self.isSucceeded = 0                               
                tmp = self._path[-1]
                self.goal = np.array([tmp[0],tmp[1]])
                self.g_th = tmp[2]
                self.timeout = tmp[3]
                self.initialized = False
        elif self.isSucceeded == 0:
            if self.clk - self.base_time > self.timeout:
                rospy.logwarn("Failed [time out] : {} {} {} {}".format(self.goal[0],self.goal[1],self.g_th,self.timeout))                
                if self.retry:
                    self.isSucceeded = -1
                    rospy.loginfo("RETRY!")
                else:
                    self.isSucceeded = 1
                            
    # 目的地を追加する
    def put_goal(self,x,y,th,timeout = 25.0):        
        self._queue.append([x,y,th,timeout])
        if timeout is None:
            timeout = "-"
        else:
            timeout = "{:.1f}".format(timeout)

        if th is None:
            th = "None"
        else:
            th = "{:.03f}".format(th)
        
        rospy.loginfo("put goal : [{:.03f} {:.03f} {} {}]".format(x,y,th,timeout))    
           
    # 目的地を強制変更する
    def set_goal(self,x,y,th,timeout=20.0):        
        # キューをすべて削除        
        self._queue = []
        # 強制的に目指すべき場所を変更する
        self.goal = np.array([x,y])
        self.g_th = th
        # タイムアウト秒数を指定する
        self.timeout = timeout

    # 目的のターゲットを設定する
    def set_target(self,target_name,timeout=20.0):        
        # 目的のターゲットの位置を取得する
        xy,th,row,col = getTargetPose(target_name)

        # 経路を考える
        
        
        # 1.現在地と目的地の左右関係を確かめる        
        #       mode[0] : 目的地が自機より左側 -> 中継点は右側
        #       mode[1] : 目的地が自機より上側 -> 中継点は下側
        mode = [None,None]
        mode[0] = self.trans[0] >= xy[0]
        mode[1] = self.trans[1] >= xy[1]        

        # 2. 目的地付近の中継点の行/列を考える
        
        # mode[0] : 目的地が自機より左側 -> 中継点は右側
        if mode[0]:
            ref_c = col[1]
        else:
            ref_c = col[0]

        # mode[1] : 目的地が自機より上側 -> 中継点は下側
        ref_r = row[0]
        if len(row) > 1 and not mode[1]:            
            ref_r = row[1]
        
        # 3. 現在地の行/列 番号を考える
        cur_r, cur_c = getRC(self.trans[0],self.trans[1])
        
        print("cur : {} {}".format(cur_r,cur_c))
        print("ref : {} {}".format(ref_r,ref_c))

        # 4. 中継点を追加する
        if cur_r < ref_r:
            r_step = 1            
        else:
            r_step = -1
        
        if cur_c < ref_c:
            c_step = 1
        else:
            c_step = -1

        # self.stop()
        point = getPoint(cur_r,cur_c)
        self.put_goal(point[0],point[1],None)

        for c in range(cur_c,ref_c+c_step,c_step):
            point = getPoint(cur_r,c)
            self.put_goal(point[0],point[1],None)   

        for r in range(cur_r,ref_r+r_step,r_step):
            point = getPoint(r,ref_c)
            self.put_goal(point[0],point[1],None)        
        
        # 5. 目標値を追加する
        self.put_goal(xy[0],xy[1],th)



    # 強制的に止める
    def stop(self):
        self.set_goal(None,None,None,0)
        self.nearest_target_name = None

    # ゴールするまで待つ
    def wait_goal(self,timeout=20):
        base_time = self.clk
        while self.clk-base_time < timeout:
            if self.isSucceeded == 1:
                return
            time.sleep(0.05)
    def wait(self,time):
        base_time = self.clk
        while self.clk-base_time < time:
            time.sleep(0.05)
        return        

if __name__ == '__main__':
    rospy.init_node('control_node')
    bot = ControlBot(version)
    bot.strategy()


