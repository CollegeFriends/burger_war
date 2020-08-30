#!/home/yodai/anaconda3/envs/CollegeFriends/bin/python

import io
import math

import matplotlib.pyplot as plt
import numpy as np
import tensorflow as tf

import rospy
from geometry_msgs.msg import Pose
from keras.callbacks import CSVLogger, History, ModelCheckpoint
from keras.layers import (Activation, BatchNormalization, Concatenate, Conv2D,
                          Dense, Dropout, Flatten, Input, Reshape)
from keras.layers.advanced_activations import PReLU
from keras.layers.convolutional import MaxPooling2D
from keras.models import Model, Sequential, load_model, model_from_json
from keras.optimizers import (SGD, Adadelta, Adagrad, Adam, Adamax, Nadam,
                              RMSprop)
from keras.regularizers import l2
from keras.utils import np_utils, plot_model
from sensor_msgs.msg import LaserScan


class Localization():
    def __init__(self):

        self.is_model_sw = False
        self.np_position_pre = np.array([0.0, -1.3, 1.57]).astype("float32")
        self.np_position = np.zeros(3).astype("float32")

        self.model = load_model('burger_war/scripts/keras/model.h5')
        self.model.summary()

        # self.tf_sub = rospy.Subscriber("/tf", TFMessage, self.tf_callback)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        # self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1000)
        self.loc_pub = rospy.Publisher('location', Pose, queue_size=1)

        self.strategy()

    def scan_callback(self, scan):
        # print(scan.ranges[0])
        np_scan = np.array(scan.ranges).astype("float32")
        np_scan = np.where(np_scan == np.inf, 3.5, np_scan)
        scan_tmp = np.expand_dims(np_scan, -1)
        position_quadrant = 0

        if self.np_position_pre[0] >= 0 and self.np_position_pre[1] >= 0:
            position_quadrant = 1
        elif self.np_position_pre[0] < 0 and self.np_position_pre[1] >= 0:
            position_quadrant = 2
        elif self.np_position_pre[0] < 0 and self.np_position_pre[1] < 0:
            position_quadrant = 3
        elif self.np_position_pre[0] >= 0 and self.np_position_pre[1] < 0:
            position_quadrant = 4

        # print(position_quadrant)

        if position_quadrant == 2:
            scan_tmp = np.concatenate([scan_tmp[0:1], scan_tmp[-1:0:-1]])
        elif position_quadrant == 4:
            scan_tmp = np.concatenate([scan_tmp[0:1], scan_tmp[-1:0:-1]])

        if position_quadrant == 2:
            scan_tmp = np.concatenate([scan_tmp[0:1], scan_tmp[-1:0:-1]])
            self.np_position_pre[0] *= -1
            if self.np_position_pre[2] >= 0:
                self.np_position_pre[2] = -self.np_position_pre[2] + np.pi
            else:
                self.np_position_pre[2] = -self.np_position_pre[2] - np.pi
        elif position_quadrant == 3:
            self.np_position_pre[0] *= -1
            self.np_position_pre[1] *= -1
            if self.np_position_pre[2] >= 0:
                self.np_position_pre[2] = self.np_position_pre[2] - np.pi
            else:
                self.np_position_pre[2] = self.np_position_pre[2] + np.pi
        elif position_quadrant == 4:
            scan_tmp = np.concatenate([scan_tmp[0:1], scan_tmp[-1:0:-1]])
            self.np_position_pre[1] *= -1
            self.np_position_pre[2] *= -1

        angle = np.arctan2(self.np_position_pre[0], self.np_position_pre[1])
        position_lu = 0
        if self.np_position_pre[0] >= self.np_position_pre[1]:
            position_lu = 0
        else:
            position_lu = 1
        position_tmp = self.model.predict([np.expand_dims(scan_tmp, 0), np.expand_dims(position_lu, 0)])[0]

        # position_tmp = self.model.predict(np.expand_dims(scan_tmp, 0))[0]

        if position_quadrant == 2:
            position_tmp[0] *= -1
            if position_tmp[2] >= 0:
                position_tmp[2] = -position_tmp[2] + np.pi
            else:
                position_tmp[2] = -position_tmp[2] - np.pi
        elif position_quadrant == 3:
            position_tmp[0] *= -1
            position_tmp[1] *= -1
            if position_tmp[2] >= 0:
                position_tmp[2] = position_tmp[2] - np.pi
            else:
                position_tmp[2] = position_tmp[2] + np.pi
        elif position_quadrant == 4:
            position_tmp[1] *= -1
            position_tmp[2] *= -1

        self.np_position = position_tmp
        print(self.np_position)
        # if not self.is_model_sw:
        #     self.model = load_model('burger_war/scripts/keras/model.h5')
        #     # self.model._make_predict_function()
        #     self.model.summary()
        #     self.is_model_sw = True
        # self.np_position = self.model.predict(np.expand_dims(scan_tmp, 0))[0]
        # print(self.np_position)
        # print("=" * 10)

        self.np_position_pre = self.np_position

    def strategy(self):
        r = rospy.Rate(10)  # change speed 1fps

        while not rospy.is_shutdown():
            pose = Pose()
            pose.position.x = self.np_position_pre[0]
            pose.position.y = self.np_position_pre[1]
            self.loc_pub.publish(pose)
            r.sleep()


if __name__ == "__main__":
    rospy.init_node("localization")
    localization = Localization()
    rospy.spin()
