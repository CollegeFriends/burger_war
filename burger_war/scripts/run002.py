import rospy

from geometry_msgs.msg import Twist
from burger_war.msg import CvCam


class RandomBot():

    def __init__(self, bot_name="NoName"):
        self.seq = 0
        self.name = bot_name
        self.cvcam_sub = rospy.Subscriber('cv_cam', CvCam, self.cvcam_callback)
        self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1000)
        self.cvcam = CvCam()
        self.cvcam.red_center = [0.0] * 2

    def cvcam_callback(self, cvcam):
        self.cvcam = cvcam

    def calcTwist(self):

        twist = Twist()

        # print(self.seq)
        # if self.seq == 0:
        #     twist.angular.z = 1
        #     if red_center[0] >= 0 and red_center[1] >= 0:
        #         self.seq = 1
        # elif self.seq == 1:
        #     print("")
        # else:
        #     print("")

        # self.twist_pub.publish(twist)
        Kp = 0.005
        Ki = 0.0
        Kd = 0.00005
        integral = 0.0
        error = 0.0
        error_pre = 0.0
        dt = 0.1
        r = rospy.Rate(1.0 / dt)
        anglar_z_PID = 0.0
        target_001 = 320.0
        while not rospy.is_shutdown():
            error_pre = error
            if self.cvcam.red_center[0] >= 0:
                error = target_001 - self.cvcam.red_center[0]
            anglar_z_PID = Kp * error + Kd * (error - error_pre) / dt
            twist.angular.z = anglar_z_PID
            self.twist_pub.publish(twist)
            r.sleep()


if __name__ == '__main__':
    rospy.init_node('random_run')
    bot = RandomBot('Random')
    bot.calcTwist()
    # rospy.spin()
    # bot.strategy()
