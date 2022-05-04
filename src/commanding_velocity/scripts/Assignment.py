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
        #Twist variable
        self.twist = Twist()
        
        
        #Subscriber for gettng the image data
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image, self.visualise_colours)
        #Laserscan for detecting walls
        self.scan_area_sub = rospy.Subscriber('/scan', LaserScan, self.Movement)
       #Movement publisher
        self.publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)

        #testing tool
        print("intialising")

    def visualise_colours(self, data):

        #create the image window
        namedWindow("Image window")
        namedWindow("mask")
        namedWindow("HSV")
        #Convert the sensor data to op[en cv imge using the bridge and store in variable
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError:
            print("error")

        #create another colour space for indetifying colours
        hsv_img = cvtColor(self.cv_image, COLOR_BGR2HSV)
        img = cvtColor(self.cv_image, COLOR_BGR2HSV)
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

        mask = cv2.inRange(self.cv_image, low_all, all)

        #mask = cv2.bitwise_and(self.cv_image, self.cv_image, mask=mask)
        #crete  mask lookign for colours between the ranges given
        self.identify_red = cv2.inRange(self.cv_image, low_red, red)
        identify_green = cv2.inRange(self.cv_image, low_green, green)
        identify_blue = cv2.inRange(self.cv_image, darkblue, blue)
        self.identify_yellow = cv2.inRange(self.cv_image, lower_yellow, upper_yellow)

        #getting the dimensions of the image
        self.x, self.y, self.z = self.cv_image.shape
        #define self.twist variable
        

        #Search the image
        #height of the search bar
        search_top = 3*self.x/5
        #width of search area
        search_bot = search_top + 20

        print(search_bot, search_top, "search values")
        print(self.x, self.y, self.z, "dimensions")

        #cuts out part of the image so that the robot isnt identified
        mask[0:search_top, 0:self.y] = 0
        mask[search_bot:self.x, 0:self.y] = 0

        #cuts out part of the image so that the robot isnt identified
        #first 2 numbers is the height of the rectangle
        self.identify_yellow[0:400, 0:640] = 0

        self.identify_yellow[400:0, 640:0] = 0

        #create variable for identifying moments in the given mask
        self.R = cv2.moments(self.identify_red)
        self.G = cv2.moments(identify_green)
        self.B = cv2.moments(identify_blue)
        self.Y = cv2.moments(self.identify_yellow)


        #if self.R['m00'] > 0:
            #captures certain moments of the image
        #    cx = int(R['m10']/R['m00'])
        #    cy = int(R['m01']/R['m00'])
            #draw circle on the screen in the middle of the mask
            #cv2.circle(self.cv_image, (cx, cy), 20, (0, 0, 255), -1)
        #    print("red - turning")

            #the error rate for moving away from the colour
        #    err = cx - y/2
            #Move the bot forward
        #    self.twist.linear.x = 0.2
            #turn the bot away from the colour (by using the error rate / 100)
        #    self.twist.angular.z = float(err) / 100
            #print the turning angle to cosole
        #    print(self.twist.angular.z)
            #publish the self.twist movement

        #elif self.G['m00'] > 0:
            # 
        #    cx = int(G['m10']/G['m00'])
        #    cy = int(G['m01']/G['m00'])
        #    #cv2.circle(self.cv_image, (cx, cy), 20, (0, 0, 255), -1)
        #    print("green - moving to")
        #    #the error rate for moving to the colour
        #    err = cx - y/2
        #    self.twist.linear.x = 0.2
        #    self.twist.angular.z = -float(err) / 100
        #    print(self.twist.angular.z)
            #self.publisher.publish(self.twist)

        
        #else:
            #self.twist.angular.z = 0.2
            #self.publisher.publish(self.twist)
        #display the image window with the open cv image
        
        imshow("mask", mask)
        imshow("Image window", self.cv_image)
        imshow("HSV", self.identify_yellow)
        waitKey(1)

    def Movement(self, laser_msg):

        blue_on = False

        if laser_msg.ranges[50] > 1.0:
            
            print("Searching!")
            self.twist.angular.z = 0.2
            self.publisher.publish(self.twist)

            if self.Y['m00'] > 0:
                
                cx = int(self.Y['m10']/self.Y['m00'])
                cy = int(self.Y['m01']/self.Y['m00'])
                cv2.circle(self.cv_image, (cx, cy), 20, (0, 0, 255), -1)
                print("yellow - following line")
                #the error rate for moving to the colour
                err = cx - self.y/2
                self.twist.linear.x = 0.2
                self.twist.angular.z = -float(err) / 200
                print(self.twist.angular.z, "Angular speed")
                print(self.twist.linear.x, "linear speed")
                self.publisher.publish(self.twist)

                if self.B['m00'] > 0:
                    
                    blue_on = True

                    cx = int(self.B['m10']/self.B['m00'])
                    cy = int(self.B['m01']/self.B['m00'])
                    cv2.circle(self.cv_image, (cx, cy), 20, (0, 0, 255), -1)
                    print("BLUE! - moving to")
                    ##the error rate for moving to the colour
                    err = cx - self.y/2
                    print(err, "error rate")

                    self.twist.linear.x = 0.2
                    self.twist.angular.z = -float(err) / 150
                    print(self.twist.angular.z)
                    self.publisher.publish(self.twist)
        else:
            self.twist.linear.x = 0
            self.twist.angular.z = 0.2
            self.publisher.publish(self.twist)
            print("Walllll!")
            blue_on = False


            
            
                
       

rospy.init_node('run', anonymous=True)
run = MoveColour()
rospy.spin()