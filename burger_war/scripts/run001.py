import rospy

from geometry_msgs.msg import Twist
from burger_war.msg import CvCam


class RandomBot():

    def __init__(self, bot_name="NoName"):
        self.seq = 0
        self.name = bot_name
        self.cvcam_sub = rospy.Subscriber('cv_cam', CvCam, self.cvcam_callback)
        self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1000)

    def cvcam_callback(self, cvcam):
        print(cvcam.red_center)
        print("")
        self.calcTwist(cvcam.red_center)

    def calcTwist(self, red_center):

        twist = Twist()

        print(self.seq)
        if self.seq == 0:
            twist.angular.z = 1
            if red_center[0] >= 0 and red_center[1] >= 0:
                self.seq = 1
        elif self.seq == 1:
            print("")
        else:
            print("")

        self.twist_pub.publish(twist)


if __name__ == '__main__':
    rospy.init_node('random_run')
    bot = RandomBot('Random')
    rospy.spin()
    # bot.strategy()
