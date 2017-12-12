#!/usr/bin/env python
import sys
import numpy as np
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
    def __init__(self):
        self.image_pub = rospy.Publisher("processed_image", Image)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.callback)

        self.boundaries = [
            ([17, 15, 100], [50, 56, 200]), # R
            ([86, 31, 4], [220, 88, 50]), # B
            ([25, 146, 190], [62, 174, 250]), # Y
            # TODO: Insert green in here too
        ]

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

        (rows, cols, channels) = cv_image.shape
        # TODO: report centroids for ea. color region shown the sort
        # laterally (which comes first)
        for (lower, upper) in self.boundaries:
            lower = np.array(lower, dtype="uint8")
            upper = np.array(upper, dtype="uint8")

            mask = cv2.inRange(cv_image, lower, upper)
            output = cv2.bitwise_and(cv_image, cv_image, mask=mask)

            imageOut = np.hstack([cv_image, output])

        cv2.imshow("Image Window", imageOut)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(imageOut, "bgr8"))
        except CvBridgeError as e:
            print e

def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting Down"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

