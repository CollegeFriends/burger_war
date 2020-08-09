#!/usr/bin/env python
# -*- coding: utf-8 -*-
# python burger_war/scripts/print2.py

import math
import sys

import numpy as np

import rospy
import tf
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


class PrintMsg():
    def __init__(self):
        self.tf_translation = []
        self.tf_rotation_quaternion = []
        self.tf_rotation_euler = []

        self.position_i = []
        self.position_pre_i = []
        self.scan_i = []
        self.start_cnt = 0

        if 0:  # 新規
            self.np_scan = np.zeros((0, 360)).astype("float32")
            self.np_position = np.zeros((0, 3)).astype("float32")
            self.np_position_pre = np.zeros((0, 3)).astype("float32")
        else:  # 読込
            self.np_scan = np.load("burger_war/scripts/keras/data/scan.npy")
            self.np_position = np.load("burger_war/scripts/keras/data/position.npy")
            self.np_position_pre = np.load("burger_war/scripts/keras/data/position_pre.npy")

        self.tf_sub = rospy.Subscriber("/tf", TFMessage, self.tf_callback)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        # self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1000)

    def tf_callback(self, tf_topic):
        # print(tf_topic)
        if tf_topic.transforms[0].child_frame_id == "base_footprint":
            self.tf_translation = [
                tf_topic.transforms[0].transform.translation.x,
                tf_topic.transforms[0].transform.translation.y,
                tf_topic.transforms[0].transform.translation.z
            ]
            self.tf_rotation_quaternion = [
                tf_topic.transforms[0].transform.rotation.x,
                tf_topic.transforms[0].transform.rotation.y,
                tf_topic.transforms[0].transform.rotation.z,
                tf_topic.transforms[0].transform.rotation.w
            ]
            self.tf_rotation_euler = tf.transformations.euler_from_quaternion(self.tf_rotation_quaternion)

    def scan_callback(self, scan):
        # print(scan.ranges[0])
        # print(self.tf_translation)
        # print(self.tf_rotation_euler)
        # print("translation: [{:.3f}, {:.3f}]".format(self.tf_translation[0], self.tf_translation[1]))
        # print("rotation: {:.3f}".format(self.tf_rotation_euler[2]*180.0 / math.pi))
        # print("=" * 10)
        if self.start_cnt < 10:
            self.start_cnt += 1
            self.position_i = np.array([self.tf_translation[0], self.tf_translation[1], self.tf_rotation_euler[2]]).astype("float32")
            self.position_pre_i = self.position_i
            return

        self.scan_i = np.array([scan.ranges]).astype("float32")
        self.scan_i = np.where(self.scan_i == np.inf, 3.5, self.scan_i)

        self.position_i = np.array([self.tf_translation[0], self.tf_translation[1], self.tf_rotation_euler[2]]).astype("float32")
        print(self.position_i)

        self.np_scan = np.vstack([self.np_scan, self.scan_i])
        self.np_position = np.vstack([self.np_position, self.position_i])
        self.np_position_pre = np.vstack([self.np_position_pre, self.position_pre_i])

        # np.save("burger_war/scripts/keras/data/scan.npy", self.np_scan)
        # np.save("burger_war/scripts/keras/data/position.npy", self.np_position)
        # np.save("burger_war/scripts/keras/data/position_pre.npy", self.np_position_pre)

        self.position_pre_i = self.position_i

    def loop(self):
        while not rospy.is_shutdown():
            key_input = raw_input()
            if key_input == "q":
                sys.exit()
            np_scan_tmp = np.copy(self.np_scan)
            np_position_tmp = np.copy(self.np_position)
            np_position_pre_tmp = np.copy(self.np_position_pre)
            print(len(np_scan_tmp))
            print(len(np_position_tmp))
            print(len(np_position_pre_tmp))
            if len(np_scan_tmp) == len(np_position_tmp) and len(np_scan_tmp) == len(np_position_pre_tmp):
                np.save("burger_war/scripts/keras/data/scan.npy", np_scan_tmp)
                np.save("burger_war/scripts/keras/data/position.npy", np_position_tmp)
                np.save("burger_war/scripts/keras/data/position_pre.npy", np_position_pre_tmp)
                print("save OK!")
                sys.exit()


if __name__ == "__main__":
    rospy.init_node("print2")
    print_msg = PrintMsg()
    print_msg.loop()
    # rospy.spin()
