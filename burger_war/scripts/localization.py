#!/root/anaconda3/envs/CollegeFriends/bin/python

import math

import numpy as np
import tensorflow as tf
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

import rospy
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

    def scan_callback(self, scan):
        # print(scan.ranges[0])
        np_scan = np.array(scan.ranges).astype("float32")
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

        if position_quadrant == 2:
            scan_tmp = np.concatenate([scan_tmp[0:1], scan_tmp[-1:0:-1]])
        elif position_quadrant == 4:
            scan_tmp = np.concatenate([scan_tmp[0:1], scan_tmp[-1:0:-1]])

        # if not self.is_model_sw:
        #     self.model = load_model('burger_war/scripts/keras/model.h5')
        #     # self.model._make_predict_function()
        #     self.model.summary()
        #     self.is_model_sw = True
        self.np_position = self.model.predict(np.expand_dims(scan_tmp, 0))[0]
        print(self.np_position)
        # print("=" * 10)

        self.np_position_pre = self.np_position


if __name__ == "__main__":
    rospy.init_node("localization")
    localization = Localization()
    rospy.spin()
