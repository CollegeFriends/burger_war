import rospy
from geometry_msgs.msg import Twist
from aruco_msgs.msg import MarkerArray
from burger_war.msg import CvCam


class Print():

    def __init__(self):
        rospy.init_node("print")
        rospy.Subscriber('cmd_vel', Twist, self.twist_callback)
        rospy.Subscriber('target_id', MarkerArray, self.markerarray_callback)

    def twist_callback(self, twist):
        print(twist.linear)
        print("")

    def markerarray_callback(self, markerarray):
        print(markerarray.markers[0].id)
        print("")


if __name__ == "__main__":
    Print()
    rospy.spin()
