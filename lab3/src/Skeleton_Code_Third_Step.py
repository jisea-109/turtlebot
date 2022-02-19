#!/usr/bin/env python

#Exercise 3 - If green object is detected, and above a certain size, then send a message (print or use lab1)

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

        # Initialise any flags that signal a colour has been detected (default to false)
        self.flag = False

        # Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)
        self.sensitivity = 10

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
        # Set the upper and lower bounds for the colour you wish to identify - green
        hsv_green_lower = np.array([60-self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60+self.sensitivity, 255, 255])
        # Convert the rgb image into a hsv image
        Hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # Filter out everything but a particular colour using the cv2.inRange() method
        mask = cv2.inRange(Hsv_image, hsv_green_lower, hsv_green_upper)
        # Apply the mask to the original image using the cv2.bitwise_and() method
        bitwiseAnd = cv2.bitwise_and(cv_image,cv_image,mask = mask)


        # Find the contours that appear within the certain colour mask using the cv2.findContours() method
        # For <mode> use cv2.RETR_LIST for <method> use cv2.CHAIN_APPROX_SIMPLE
        contours, hierarchy = cv2.findContours(mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            # Loop over the contours
            # There are a few different methods for identifying which contour is the biggest:
            # Loop through the list and keep track of which contour is biggest or
            # Use the max() method to find the largest contour
            #c = max(<contours>, key=cv2.contourArea)
            c = max(contours, key=cv2.contourArea)
            #Moments can calculate the center of the contour
            M = cv2.moments(c)
            cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
            #Check if the area of the shape you want is big enough to be considered
            # If it is then change the flag for that colour to be True(1)
            if cv2.contourArea(c) > 2500: #<What do you think is a suitable area?>:
                # draw a circle on the contour you're identifying
                #minEnclosingCircle can find the centre and radius of the largest contour(result from max())
                (x, y), radius = cv2.minEnclosingCircle(c)
                center = (int(x),int(y))
                radius = int(radius)
                #cv2.circle(<image>,(<center x>,<center y>),<radius>,<colour (rgb tuple)>,<thickness (defaults to 1)>)
                cv2.circle(Hsv_image,center,radius,hsv_green_upper,1)

                # Then alter the values of any flags
                self.flag = True
        else:
            self.flag = False
        #if the flag is true (colour has been detected)
            #print the flag or colour to test that it has been detected
            #alternatively you could publish to the lab1 talker/listener
        if self.flag == True:
            print("Green is detected")
        else:
            print("Green is not detected")
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
