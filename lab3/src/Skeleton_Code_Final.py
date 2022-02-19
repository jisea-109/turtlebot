#!/usr/bin/env python

# Exercise 4 - following a colour (green) and stopping upon sight of another (blue).

from __future__ import division
import cv2
import numpy as np
import rospy
import sys

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class colourIdentifier():

    def __init__(self):
        # Initialise a publisher to publish messages to the robot base
        # We covered which topic receives messages that move the robot in the 2nd Lab Session
        self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist)
        # Initialise any flags that signal a colour has been detected (default to false)
        self.colour1_flag = 0
        self.colour2_flag = 0

        # Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)
        self.sensitivity = 10

        # Initialise some standard movement messages such as a simple move forward and a message with all zeroes (stop)
        self.desired_velocity = Twist()
        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)
        # We covered which topic to subscribe to should you wish to receive image data

    def callback(self, data):
        # Convert the received image into a opencv image
        # But remember that you should always wrap a call to this conversion method in an exception handler
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError:
            pass

        # Set the upper and lower bounds for the two colours you wish to identify
        #hue value = 0 to 179
        hsv_colour1_lower = np.array([60 - self.sensitivity, 100, 100])
        hsv_colour1_upper = np.array([60 + self.sensitivity, 255, 255])
        hsv_colour2_lower = np.array([100, 245 - self.sensitivity, 100])
        hsv_colour2_upper = np.array([255, 245 + self.sensitivity, 255])

        # Convert the rgb image into a hsv image
        Hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Filter out everything but a particular colour using the cv2.inRange() method
        mask = cv2.inRange(Hsv_image, hsv_colour1_lower, hsv_colour1_upper)
        mask_blue = cv2.inRange(Hsv_image, hsv_colour2_lower, hsv_colour2_upper)
        # Apply the mask to the original image using the cv2.bitwise_and() method
        # As mentioned on the worksheet the best way to do this is to bitwise and an image with itself and pass the mask to the mask parameter
        bitwiseAnd = cv2.bitwise_and(cv_image,cv_image,mask = mask)
        bitwiseAnd_blue = cv2.bitwise_and(cv_image,cv_image,mask = mask_blue)
        # Find the contours that appear within the certain colour mask using the cv2.findContours() method
        # For <mode> use cv2.RETR_LIST for <method> use cv2.CHAIN_APPROX_SIMPLE
        greencontours, hierarchy = cv2.findContours(mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        # Loop over the contours
        bluecontours, hierarchy_blue = cv2.findContours(mask_blue,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        if len(greencontours)>0:

            # There are a few different methods for identifying which contour is the biggest
            # Loop through the list and keep track of which contour is biggest or
            # Use the max() method to find the largest contour
            #c = max(<contours>, key=cv2.contourArea)
            c = max(greencontours, key=cv2.contourArea)

            #Check if the area of the shape you want is big enough to be considered
            # If it is then change the flag for that colour to be True(1)
            if cv2.contourArea(c) > 2000: #<What do you think is a suitable area?>:
                (x, y), radius = cv2.minEnclosingCircle(c)
                center = (int(x),int(y))
                radius = int(radius)
                cv2.circle(Hsv_image,center,radius,hsv_colour1_upper,1)
                # Alter the value of the flag
                self.colour1_flag = 1
        else:
            self.colour1_flag = 0
        if len(bluecontours) > 0:
            c = max(bluecontours, key=cv2.contourArea)

            #Check if the area of the shape you want is big enough to be considered
            # If it is then change the flag for that colour to be True(1)
            if cv2.contourArea(c) > 2000: #<What do you think is a suitable area?>:
                (x, y), radius = cv2.minEnclosingCircle(c)
                center = (int(x),int(y))
                radius = int(radius)
                cv2.circle(Hsv_image,center,radius,hsv_colour2_upper,1)
                self.colour2_flag = 1
        else:
            self.colour2_flag = 0
        #Check if a flag has been set = colour object detected - follow the colour object
        if self.colour1_flag == 1:
            if self.colour2_flag == 1:
                self.desired_velocity.linear.x = 0
            else:
                if cv2.contourArea(c) > 150000:
                    # Too close to object, need to move backwards
                    self.desired_velocity.linear.x = -0.3

                elif cv2.contourArea(c) < 150000:
                    # Too far away from object, need to move forwards
                    self.desired_velocity.linear.x = +0.3
                else:
                    pass
                #self.desired_velocity.linear.x = 0
        else:
            self.desired_velocity.linear.x = 0
        # Be sure to do this for the other colour as well
        #Setting the flag to detect blue, and stop the turtlebot from moving if blue is detected

        # Publish moves
        self.pub.publish(self.desired_velocity)
        #Show the resultant images you have created. You can show all of them or just the end result if you wish to.
        cv2.imshow('Third_Step', bitwiseAnd)
        cv2.waitKey(3)
# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main(args):
    # Instantiate your class
    # And rospy.init the entire node
    cI = colourIdentifier()
    rospy.init_node('Skeleton__', anonymous=True)
    rate = rospy.Rate(10)
    try:
        rospy.spin()
    except KeyboardInterrupts:
        print("Shutting down")
    cv2.destroyAllWindows()
    # Ensure that the node continues running with rospy.spin()
    # You may need to wrap it in an exception handler in case of KeyboardInterrupts
    # Remember to destroy all image windows before closing node

# Check if the node is executing in the main path
if __name__ == '__main__':
    main(sys.argv)
