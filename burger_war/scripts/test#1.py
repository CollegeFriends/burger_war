from test_util import *
from controlBot import ControlBot

reset_sim()
a = ControlBot()       
rospy.init_node('control_node')     
a.strategy(timeout=10.0)