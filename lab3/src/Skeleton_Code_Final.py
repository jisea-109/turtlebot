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


        # Initialise any flags that signal a colour has been detected (default to false)


        # Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)


        # Initialise some standard movement messages such as a simple move forward and a message with all zeroes (stop)

        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use

        # We covered which topic to subscribe to should you wish to receive image data

    def callback(self, data):
        # Convert the received image into a opencv image
        # But remember that you should always wrap a call to this conversion method in an exception handler


        # Set the upper and lower bounds for the two colours you wish to identify
        #hue value = 0 to 179
        hsv_colour1_lower = np.array([<Hue value> - self.sensitivity, 100, 100])
        hsv_colour1_upper = np.array([<Hue value> + self.sensitivity, 255, 255])
        #hsv_colour2_lower = np.array([<Hue value> - self.sensitivity, 100, 100])
        #hsv_colour2_upper = np.array([<Hue value> + self.sensitivity, 255, 255])

        # Convert the rgb image into a hsv image


        # Filter out everything but a particular colour using the cv2.inRange() method


        # Apply the mask to the original image using the cv2.bitwise_and() method
        # As mentioned on the worksheet the best way to do this is to bitwise and an image with itself and pass the mask to the mask parameter


        # Find the contours that appear within the certain colour mask using the cv2.findContours() method
        # For <mode> use cv2.RETR_LIST for <method> use cv2.CHAIN_APPROX_SIMPLE

        # Loop over the contours
        #if len(greencontours)>0:

            # There are a few different methods for identifying which contour is the biggest
            # Loop through the list and keep track of which contour is biggest or
            # Use the max() method to find the largest contour
            #c = max(<contours>, key=cv2.contourArea)


            #Check if the area of the shape you want is big enough to be considered
            # If it is then change the flag for that colour to be True(1)
            if cv2.contourArea(c) > x #<What do you think is a suitable area?>:

                # Alter the value of the flag


        #Check if a flag has been set = colour object detected - follow the colour object
        if self.colour1_flag == 1:
            if cv2.contourArea(c) > ****:
                # Too close to object, need to move backwards


            else if cv2.contourArea(c) < ****:
                # Too far away from object, need to move forwards

            else:


        # Be sure to do this for the other colour as well
        #Setting the flag to detect blue, and stop the turtlebot from moving if blue is detected

        # Publish moves

        #Show the resultant images you have created. You can show all of them or just the end result if you wish to.

# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main(args):
    # Instantiate your class
    # And rospy.init the entire node
    cI = colourIdentifier()
    # Ensure that the node continues running with rospy.spin()
    # You may need to wrap it in an exception handler in case of KeyboardInterrupts
    # Remember to destroy all image windows before closing node

# Check if the node is executing in the main path
if __name__ == '__main__':
    main(sys.argv)
