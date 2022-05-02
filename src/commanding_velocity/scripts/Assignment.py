import rospy
import cv2
from cv2 import COLOR_BGR2HSV, waitKey, namedWindow, cvtColor, imshow,findContours,RETR_TREE, CHAIN_APPROX_SIMPLE
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
        
        self.scan_area_sub = rospy.Subscriber('/scan', LaserScan, self.Dodge_walls)

        print("intialising")

    def visualise_colours(self, data):

        #create the image window
        namedWindow("Image window")
        namedWindow("mask")
        namedWindow("HSV")
        #Convert the sensor data to op[en cv imge using the bridge and store in variable
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError:
            print("error")

        #create another colour space for indetifying colours
        hsv_img = cvtColor(cv_image, COLOR_BGR2HSV)
        img = cvtColor(cv_image, COLOR_BGR2HSV)
        #numpy arry for colour ranges 
        #all
        all = numpy.array([255, 255, 255])
        low_all = numpy.array([0, 0, 0])

        #red
        red = numpy.array([0, 0, 255])
        low_red = numpy.array([0, 0, 0])

        green = numpy.array([0, 255, 0])
        low_green = numpy.array([0, 100, 0])
        #blue
        blue = numpy.array([255, 0, 0])
        darkblue = numpy.array([100, 0, 0])
        #yellow
        lower_yellow = numpy.array([10, 60, 170])
        upper_yellow = numpy.array([255, 255, 255])

        mask = cv2.inRange(cv_image, low_all, all)

        #mask = cv2.bitwise_and(cv_image, cv_image, mask=mask)
        #crete  mask lookign for colours between the ranges given
        self.identify_red = cv2.inRange(cv_image, low_red, red)
        identify_green = cv2.inRange(cv_image, low_green, green)
        identify_blue = cv2.inRange(hsv_img, darkblue, blue)
        identify_yellow = cv2.inRange(hsv_img, lower_yellow, upper_yellow)

        #getting the dimensions of the image
        x, y, z = cv_image.shape
        #define twist variable
        twist = Twist()

        #Search the image
        #height of the search bar
        search_top = 3*x/5
        #width of search area
        search_bot = search_top + 20

        #cuts out part of the image so that the robot isnt identified
        mask[0:search_top, 0:y] = 0
        mask[search_bot:x, 0:y] = 0

        #create variable for identifying moments in the given mask
        R = cv2.moments(self.identify_red)
        G = cv2.moments(identify_green)
        B = cv2.moments(identify_blue)
        Y = cv2.moments(identify_yellow)


        if R['m00'] > 0:
            #captures certain moments of the image
            cx = int(R['m10']/R['m00'])
            cy = int(R['m01']/R['m00'])
            #draw circle on the screen in the middle of the mask
            #cv2.circle(cv_image, (cx, cy), 20, (0, 0, 255), -1)
            print("red - turning")

            #the error rate for moving away from the colour
            err = cx - y/2
            #Move the bot forward
            twist.linear.x = 0.2
            #turn the bot away from the colour (by using the error rate / 100)
            twist.angular.z = float(err) / 100
            #print the turning angle to cosole
            print(twist.angular.z)
            #publish the twist movement
            self.publisher.publish(twist)

        elif G['m00'] > 0:
            # 
            cx = int(G['m10']/G['m00'])
            cy = int(G['m01']/G['m00'])
            #cv2.circle(cv_image, (cx, cy), 20, (0, 0, 255), -1)
            print("green - moving to")
            #the error rate for moving to the colour
            err = cx - y/2
            twist.linear.x = 0.2
            twist.angular.z = -float(err) / 100
            print(twist.angular.z)
            self.publisher.publish(twist)

        elif B['m00'] > 0:
            cx = int(G['m10']/G['m00'])
            cy = int(G['m01']/G['m00'])
            #cv2.circle(cv_image, (cx, cy), 20, (0, 0, 255), -1)
            print("green - moving to")
            #the error rate for moving to the colour
            err = cx - y/2
            twist.linear.x = 0.2
            twist.angular.z = -float(err) / 100
            print(twist.angular.z)
            self.publisher.publish(twist)

        elif Y['m00'] > 0:
            cx = int(G['m10']/G['m00'])
            cy = int(G['m01']/G['m00'])
            #cv2.circle(cv_image, (cx, cy), 20, (0, 0, 255), -1)
            print("green - moving to")
            #the error rate for moving to the colour
            err = cx - y/2
            twist.linear.x = 0.2
            twist.angular.z = -float(err) / 100
            print(twist.angular.z)
            self.publisher.publish(twist)
        #else:
            #twist.angular.z = 0.2
            #self.publisher.publish(twist)
        #display the image window with the open cv image
        
        imshow("mask", self.identify_red)
        #imshow("HSV", mask)
        imshow("Image window", cv_image)
        waitKey(1)

    def Dodge_walls(self, laser_msg):

        if laser_msg.ranges[25] < 1.0:
            
            t = Twist()
            t.angular.z = 0.5
            self.publisher.publish(t)
        else:
            t = Twist()
            t.linear.x = 0.5
            self.publisher.publish(t)

rospy.init_node('run', anonymous=True)
run = MoveColour()
rospy.spin()