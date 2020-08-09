# https://qiita.com/srs/items/99d1ff2207772859763c

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from burger_war.msg import CvRect


class Cv_cam():

    def __init__(self):
        rospy.init_node("cv_cam")
        rospy.on_shutdown(self.cleanup)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("image_raw", Image, self.image_callback)
        self.cvrect_pub = rospy.Publisher("cv_rect", CvRect, queue_size=1000)

        self.rect_center = None
        self.rect_length = None
        self.rect_area = None

    def image_callback(self, ros_image):
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e
        input_image = np.array(frame, dtype=np.uint8)

        self.get_rect(input_image, True)

        cvrect = CvRect()
        [cvrect.rect_r.center, cvrect.rect_g.center, cvrect.rect_b.center] = self.rect_center
        [cvrect.rect_r.length, cvrect.rect_g.length, cvrect.rect_b.length] = self.rect_length
        [cvrect.rect_r.area, cvrect.rect_g.area, cvrect.rect_b.area] = self.rect_area

        self.cvrect_pub.publish(cvrect)
        print(cvrect)
        print("===================")
        cv2.waitKey(1)

    def get_rect(self, image, debug=False):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower = np.array([175, 10, 0])
        upper = np.array([179, 255, 255])
        mask_r1 = cv2.inRange(hsv, lower, upper)
        lower = np.array([0, 10, 0])
        upper = np.array([5, 255, 255])
        mask_r2 = cv2.inRange(hsv, lower, upper)
        mask_r = mask_r1 + mask_r2

        lower = np.array([55, 10, 0])
        upper = np.array([65, 255, 255])
        mask_g = cv2.inRange(hsv, lower, upper)

        lower = np.array([115, 10, 0])
        upper = np.array([125, 255, 255])
        mask_b = cv2.inRange(hsv, lower, upper)

        if int(cv2.__version__[0]) > 2:
            _, contours_r, _ = cv2.findContours(mask_r, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            _, contours_g, _ = cv2.findContours(mask_g, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            _, contours_b, _ = cv2.findContours(mask_b, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        elif int(cv2.__version__[0]) == 2:
            contours_r, _ = cv2.findContours(mask_r, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours_g, _ = cv2.findContours(mask_g, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours_b, _ = cv2.findContours(mask_b, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        rects_r = []
        rects_g = []
        rects_b = []
        RECT_AREA_THRESHOLD = 50
        r_area_max = RECT_AREA_THRESHOLD
        g_area_max = RECT_AREA_THRESHOLD
        b_area_max = RECT_AREA_THRESHOLD
        r_max = []
        g_max = []
        b_max = []
        r_center = (-1, -1)
        g_center = (-1, -1)
        b_center = (-1, -1)
        r_length = (0, 0)
        g_length = (0, 0)
        b_length = (0, 0)

        for contour in contours_r:
            approx = cv2.convexHull(contour)
            rect = cv2.boundingRect(approx)
            rects_r.append(rect)
            r_area = rect[2] * rect[3]
            if r_area > r_area_max:
                r_area_max = r_area
                r_center = (rect[0] + rect[2] / 2.0, rect[1] + rect[3] / 2.0)
                r_length = (rect[2], rect[3])
                r_max = rect

        for contour in contours_g:
            approx = cv2.convexHull(contour)
            rect = cv2.boundingRect(approx)
            rects_g.append(rect)
            g_area = rect[2] * rect[3]
            if g_area > g_area_max:
                g_area_max = g_area
                g_center = (rect[0] + rect[2] / 2.0, rect[1] + rect[3] / 2.0)
                g_length = (rect[2], rect[3])
                g_max = rect

        for contour in contours_b:
            approx = cv2.convexHull(contour)
            rect = cv2.boundingRect(approx)
            rects_b.append(rect)
            b_area = rect[2] * rect[3]
            if b_area > b_area_max:
                b_area_max = b_area
                b_center = (rect[0] + rect[2] / 2.0, rect[1] + rect[3] / 2.0)
                b_length = (rect[2], rect[3])
                b_max = rect

        self.rect_center = [r_center, g_center, b_center]
        self.rect_length = [r_length, g_length, b_length]
        self.rect_area = [r_length[0] * r_length[1], g_length[0] * g_length[1], b_length[0] * b_length[1]]

        if debug:
            display = image.copy()
            if r_area_max > RECT_AREA_THRESHOLD:
                cv2.rectangle(display, (r_max[0], r_max[1]), (r_max[0] + r_max[2], r_max[1] + r_max[3]), (100, 100, 255), thickness=3)
            if g_area_max > RECT_AREA_THRESHOLD:
                cv2.rectangle(display, (g_max[0], g_max[1]), (g_max[0] + g_max[2], g_max[1] + g_max[3]), (100, 255, 100), thickness=3)
            if b_area_max > RECT_AREA_THRESHOLD:
                cv2.rectangle(display, (b_max[0], b_max[1]), (b_max[0] + b_max[2], b_max[1] + b_max[3]), (255, 100, 100), thickness=3)
            cv2.imshow("make region", display)

    def cleanup(self):
        cv2.destroyAllWindows()


if __name__ == '__main__':
    Cv_cam()
    rospy.spin()
