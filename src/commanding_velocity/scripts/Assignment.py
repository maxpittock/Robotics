from calendar import TUESDAY
from math import fabs
from re import T
from sys import builtin_module_names
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
        
        self.yellow_on = False
        
        #Subscriber for gettng the image data
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image, self.visualise_colours)
        #Laserscan for detecting walls
        self.scan_area_sub = rospy.Subscriber('/scan', LaserScan, self.Movement)
       #Movement publisher
        self.publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)

        #testing tool
        print("intialising")

    def visualise_colours(self, data):

        #create the image window for displaying the opencv image
        namedWindow("Image window")
        namedWindow("red_mask")
        namedWindow("yellow_mask")
        
        try:
            #Convert the sensor data to open cv imge using the bridge and store in variable
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError:
            print("error")

        #create another colour space for indetifying colours - this is here incase i want to draw an object onto the image without
        hsv_img = cvtColor(self.cv_image, COLOR_BGR2HSV)

        
        #numpy arry for colour ranges 
        #all
        all = numpy.array([255, 255, 255])
        low_all = numpy.array([0, 0, 0])
        #red
        red = numpy.array([10, 50, 255])
        low_red = numpy.array([0, 0, 0])

        green = numpy.array([0, 255, 0])
        low_green = numpy.array([0, 100, 0])
        #blue
        blue = numpy.array([255, 0, 0])
        darkblue = numpy.array([100, 0, 0])
        #yellow
        lower_yellow = numpy.array([0, 100, 100])
        upper_yellow = numpy.array([0, 255, 255])
        #create a mask that includes all colour ranges
        mask = cv2.inRange(self.cv_image, low_all, all)
        #crete  mask lookign for colours between the ranges given - this helps me identify colours later on
        #red
        self.identify_red = cv2.inRange(self.cv_image, low_red, red)
        #green
        self.identify_green = cv2.inRange(self.cv_image, low_green, green)
        #blue
        self.identify_blue = cv2.inRange(self.cv_image, darkblue, blue)
        #yellow
        self.identify_yellow = cv2.inRange(self.cv_image, lower_yellow, upper_yellow)

        #getting the dimensions of the image
        self.x, self.y, self.z = self.cv_image.shape
        #define self.twist variable
        

        #Search the image
        #height of the search area
        search_top = 3*self.x/5
        #width of search area
        search_bot = search_top + 20

        #debugging
        #print(search_bot, search_top, "search values")
        #print(self.x, self.y, self.z, "dimensions")

        #cuts out part of the image so that the robot isnt identified
        self.identify_red[0:search_top, 0:self.y] = 0
        self.identify_red[search_bot:self.x, 0:self.y] = 0

        #cuts it so the yellow mask can only see top of image
        self.identify_yellow[0:self.x, (self.y/1):] = 0
        #create variable for identifying moments in the given mask - this allows the bot to see the colour and using this i caan make it move
        self.R = cv2.moments(self.identify_red)
        self.G = cv2.moments(self.identify_green)
        self.B = cv2.moments(self.identify_blue)
        self.Y = cv2.moments(self.identify_yellow)
        
        #display the image window with the open cv image
        imshow("red_mask", self.identify_red)
        imshow("Image window", self.cv_image)
        imshow("yellow_mask", self.identify_yellow)
        waitKey(1)

    #movement function
    def Movement(self, laser_msg):
        #wall bool to tell if the bot is near the wall
        Wall = False
          #boolean variables for colours - to tell when they have been seen
        green = False
        red = False
        #get the length variable for sensors - this can be used later when detecting walls
        j = len(laser_msg.ranges)
      
        #code for identifying walls infront of player- if there is nothing sensed within 0.5 metres then this code will do nothing
        if laser_msg.ranges[j-1] < 0.4:
            #print debug message to console - addtionally print the boolean - allows me to know if yellow line follower code is enabled or not
            print("Walllll!", self.yellow_on)
            #stops the bot from followinng yellow when it gets to close to a wall
            self.yellow_on = True
            #set wall variable to true - stops it searching
            Wall = True
            #if the bot is crrently turning left - turn left etc
            if self.twist.angular.z >= 0:
                #self.twist.linear.x = 0.0
                #turns the bot
                self.twist.angular.z = 0.5
                self.publisher.publish(self.twist)
            else:
                #self.twist.linear.x = 0.0
                self.twist.angular.z = -0.2
                self.publisher.publish(self.twist)
        #when back out of range turn theline follower code back on
        else:
            self.yellow_on = False 


        #Search for blue/colour
        #If no bool variables for colours are are then search for a colour
        if Wall == False or green == False or red == False:
            #if the robot is turning positively 
            if self.twist.linear.z >= 0:
                #setting yellow variable to true to activate the yellow colour chase
                self.yellow_on = True
                print("Searching!")
                #stop all forward momentum
                #self.twist.linear.x = 0
                #continue to turn the same way
                self.twist.angular.z = 0.2
                #publish the movements
                self.publisher.publish(self.twist)
            else: #if going oposite direction
                #self.twist.linear.x = 0
                #continue in that directions
                self.twist.angular.z = -0.2
                self.publisher.publish(self.twist)

      
    #red and green shit
    # IF there is a moment of green on the image
        if self.G['m00'] > 0:
            #turn off yellow line follow code
            self.yellow_on == True
            #set the green boolean to true so it stops searching for a colour
            green = True
            #find the middle of the colour
            cx = int(self.G['m10']/self.G['m00'])
            cy = int(self.G['m01']/self.G['m00'])
            #draw a circle where the middle is -debugging 
            #cv2.circle(self.cv_image, (cx, cy), 20, (0, 0, 255), -1)
            #debugging message
            print("green - moving to")
            #the error rate for moving to the colour
            #calculate the error rate using the colour moment data to turn away or towards
            err = cx - self.y/2
            #keep thebot moving forward
            self.twist.linear.x = 0.2
            #turn towards the goal/colour
            self.twist.angular.z = -float(err) / 100
            #print the angular value for debugging
            #print(self.twist.angular.z)
            self.publisher.publish(self.twist)

        #if there is any red on the screen eecute the cde below
        if self.R['m00'] > 0:
            #turn off the yellow follow code
            self.yellow_on == True
            #turn on the red boolean variable
            red = True
            #captures the centre of the colour
            cx = int(self.R['m10']/self.R['m00'])
            cy = int(self.R['m01']/self.R['m00'])
            #draw circle on the screen in the middle of the mask
            cv2.circle(self.cv_image, (cx, cy), 20, (0, 0, 255), -1)
            #debug message
            print("red - turning")

            #the error rate for moving away from the colour
            err = cx - self.y/2
            #slow the bot so it can avoid 
            self.twist.linear.x = 0
            #turn the bot away from the colour (by using the error rate / 100)
            self.twist.angular.z = float(err) / 100
            #print the turning angle to cosole
            print(self.twist.angular.z)
            #publish the self.twist movement
            self.publisher.publish(self.twist)
        
       
        #blue
        #if any blue on the screen
        if self.B['m00'] > 0:
            #turn on yellow finder
            #self.yellow_on = False
            yellow_on = True
            #find middle of colour
            cx = int(self.B['m10']/self.B['m00'])
            cy = int(self.B['m01']/self.B['m00'])
            #cv2.circle(self.cv_image, (cx, cy), 20, (0, 0, 255), -1)
            #debugging
            print("BLUE! - moving to")
            
            ##the error rate for moving to the colour
            err = cx - self.y/2
            #print the error rate for debugging
            #print(err, "error rate")
            #forward movement
            self.twist.linear.x = 0.2
            #angular movement with error rate - paths t blue at a slow speed
            self.twist.angular.z = -float(err) / 1300
            #ddebugging
            print(self.twist.angular.z)
            #publish the movement 
            self.publisher.publish(self.twist)
            #When blue is acvtve and the bot gets to close to the wall the yellow line follower turns on to drag the bot around the corner
            if laser_msg.ranges[j-1] <= 1.2:
                self.yellow_on = False
        
        #yellow
        #If the bool self.yellow is false
        if self.yellow_on == False:
            #and if yellow can be seen on the screen
            if self.Y['m00'] > 0:
                #self.yellow_on = False
                #Get the middle of the colour
                cx = int(self.Y['m10']/self.Y['m00'])
                cy = int(self.Y['m01']/self.Y['m00'])
               # cv2.circle(self.cv_image, (cx, cy), 20, (0, 0, 255), -1)
               #debugging
                print("yellow - following line")
                #the error rate for moving to the colour
                #error rate calculation
                err = cx - self.y/2
                #moving forwards
                self.twist.linear.x = 0.2
                #moving angualrly towards the yellow 
                self.twist.angular.z = -float(err) / 275
                #debugging
                #print(self.twist.angular.z, "Angular speed")
                #print(self.twist.linear.x, "linear speed")
                #publish the movement
                self.publisher.publish(self.twist)
        
#intialise node
rospy.init_node('run', anonymous=True)
#declare class
run = MoveColour()
#spin node
rospy.spin()