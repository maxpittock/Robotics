import rospy
import cv2
from cv2 import COLOR_BGR2HSV, waitKey, namedWindow, cvtColor, imshow, inRange,findContours,RETR_TREE, CHAIN_APPROX_SIMPLE
import numpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class MoveColour:

    def __init__(self):
        #bridge for converting ensor data to open cv image
        self.bridge = CvBridge()
        #subscriber for gathering the sensor image data from the robot
        self.image_sub = rospy.Subscriber("/image_raw",Image, self.visualise_colours)

        print("")

    def visualise_colours(self, data):
        #create the image window
        namedWindow("Image window")
        #Convert the sensor data to op[en cv imge using the bridge and store in variable
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError:
            print("error")

        #create another colour space for indetifying colours
        hsv_img = cvtColor(cv_image, COLOR_BGR2HSV)

        #numpy arry for colour ranges 
        #green
        green = numpy.array([0, 255, 0])
        dark_green = numpy.array([0, 128, 0])
        #yellow
        lower_yellow = numpy.array([10, 60, 170])
        upper_yellow = numpy.array([255, 255, 255])
        #blue
        blue = numpy.array([255, 0, 0])
        darkblue = numpy.array([128, 0, 0])
        #red
        red = numpy.array([0, 0, 255])
        dark_red = numpy.array([0, 0, 128])

        #crete  mssk lookign for colours between the ranges given
        identify_red = inRange(hsv_img, red, dark_red)

        #finds the contours within the range given
        _, red_hsv_contours, hierachy = findContours(identify_red.copy(),RETR_TREE,CHAIN_APPROX_SIMPLE)

        for c in red_hsv_contours:
            # This allows to compute the area (in pixels) of a contour
            a = cv2.contourArea(c)
            # and if the area is big enough, we draw the outline of the contour (in blue)
            if a > 100.0:
                cv2.drawContours(cv_image, c, -1, (255, 0, 0), 3)

        #display the image window with the open cv image
        imshow("Image window", cv_image)
        waitKey(1)

rospy.init_node('MoveColour', anonymous=True)
MoveColour()
rospy.spin()