import rospy
import cv2
from cv2 import COLOR_BGR2HSV, waitKey, namedWindow, cvtColor, imshow, inRange,findContours,RETR_TREE, CHAIN_APPROX_SIMPLE
import numpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class MoveColour:

    def __init__(self):
        #bridge for converting ensor data to open cv image
        self.bridge = CvBridge()
        #subscriber for gathering the sensor image data from the robot
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image, self.visualise_colours)

        #Movement publisher
        self.publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)

        print("intialising")

    def visualise_colours(self, data):

        #create the image window
        namedWindow("Image window")
        namedWindow("mask")
        #Convert the sensor data to op[en cv imge using the bridge and store in variable
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError:
            print("error")

        #create another colour space for indetifying colours
        hsv_img = cvtColor(cv_image, COLOR_BGR2HSV)
        img = cvtColor(cv_image, COLOR_BGR2HSV)
        #numpy arry for colour ranges 
        #red
        red = numpy.array([10, 50, 255])
        low_red = numpy.array([0, 0, 0])

        green = numpy.array([0, 255, 0])
        dark_green = numpy.array([0, 128, 0])
        #blue
        blue = numpy.array([255, 0, 0])
        darkblue = numpy.array([128, 0, 0])
        #yellow
        lower_yellow = numpy.array([10, 60, 170])
        upper_yellow = numpy.array([255, 255, 255])

        #crete  mask lookign for colours between the ranges given
        identify_red = inRange(hsv_img, low_red, red)
        cv2.bitwise_and(cv_image, cv_image, mask=identify_red)
        #identify_green = inRange(hsv_img, dark_green, green)
        #identify_blue = inRange(hsv_img, darkblue, blue)
        #identify_yellow = inRange(hsv_img, lower_yellow, upper_yellow)

        #finds the contours within the range given
        _, red_hsv_contours, hierachy = findContours(identify_red.copy(),RETR_TREE,CHAIN_APPROX_SIMPLE)
      #  _, green_hsv_contours, hierachy = findContours(identify_green.copy(),RETR_TREE,CHAIN_APPROX_SIMPLE)
      #  _, blue_hsv_contours, hierachy = findContours(identify_blue.copy(),RETR_TREE,CHAIN_APPROX_SIMPLE)
      #  _, yellow_hsv_contours, hierachy = findContours(identify_yellow.copy(),RETR_TREE,CHAIN_APPROX_SIMPLE)


        #getting the dimensions of the image
        x, y, z = cv_image.shape
        twist = Twist()

        #Search the image
        #height of the search bar
        search_top = 3*x/5
        #width of search area
        search_bot = search_top + 20

        #cuts out part of the image so that the robot isnt identified
        identify_red[0:search_top, 0:y] = 0
        identify_red[search_bot:x, 0:y] = 0


        M = cv2.moments(identify_red)
        if M['m00'] > 0:
            # check https://docs.opencv.org/3.4/d8/d23/classcv_1_1Moments.html
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(cv_image, (cx, cy), 20, (0, 0, 255), -1)
            print("red")

            err = cx - y/2
            twist.linear.x = 0.2
            twist.angular.z = -float(err) / 100
            print(twist.angular.z)

            self.publisher.publish(twist)

        for c in red_hsv_contours:
            # This allows to compute the area (in pixels) of a contour
            a = cv2.contourArea(c)
            # and if the area is big enough, we draw the outline
            # of the contour (in blue)
            if a > 100.0:
                cv2.drawContours(cv_image, c, -1, (255, 0, 0), 3)

        #display the image window with the open cv image
        imshow("Image window", cv_image)
        imshow("mask", identify_red)
        waitKey(1)

rospy.init_node('run', anonymous=True)
run = MoveColour()
rospy.spin()