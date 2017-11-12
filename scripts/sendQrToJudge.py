#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import requests
import json


class QrVal(object):

    def __init__(self, judge_url, side, player_name, init_code='0000'):
        # qr val subscriver
        self.qr_val_sub = rospy.Subscriber('/qr_val', String, self.qrValCallback)
        self.judge_url = judge_url
        self.historys = []
        self.side = side
        self.player_name = player_name
        self.init_code = init_code

    def sendToJudge(self, qr_val):
        data = {"name": self.player_name, "side": self.side, "id": qr_val}
        res = requests.post(self.judge_url,
                            json.dumps(data),
                            headers={'Content-Type': 'application/json'}
                            )
        return res

    def sendInitCode(self):
        try:
            res = self.sendToJudge(self.init_code)
        except:
            print("Requests Error Please Check URL " + self.judge_url)
        else:
            print("Send " + self.init_code +  "as init code To " + self.judge_url)

    def lengthTo4(self, string):
        '''
        cut or padding string length to 4
        if length is more than 4
          use last 4 char
        if length is less than 4
          padding "0"
        ex) "0123456789" -> "6789"
            "0123" -> "0123" (no change)
            "12" -> "0012"
        '''
        len = len(string)
        if len == 4:
            return string
        if len > 4:
            return string[-4:]
        elif len < 4:
            return ("0000"+string)[-4:]
        else:
            print("what happen??")
            print(string)
            return False

    def qrValCallback(self, data):
        qr_val = data.data
        qr_val = self.lengthTo4(qr_val)
        if qr_val in self.historys:
            return
        try:
            res = self.sendToJudge(qr_val)
        except:
            print("Requests Error Please Check URL " + self.judge_url)
        else:
            print("Send " + qr_val + " To " + self.judge_url)
            self.historys.append(qr_val)


if __name__ == "__main__":
    rospy.init_node("send_qr_to_judge")

    # TODO setting from launch file
    JUDGE_URL = 'http://127.0.0.1:5000/submits'
    PLAYER_NAME= 'jiro'
    SIDE = 'r'
    INIT_CODE = '0000'

    qr = QrVal(JUDGE_URL, SIDE, PLAYER_NAME, INIT_CODE)
    qr.sendInitCode()
    rospy.spin()