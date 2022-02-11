#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def publisher():
    pub = rospy.Publisher('mobile_base/commands/velocity', Twist)
    rospy.init_node('Walker', anonymous=True)
    rate = rospy.Rate(10) #10hz

    desired_velocity = Twist()

    for i in range (4):
        desired_velocity.linear.x = 0.3
        desired_velocity.angular.z = 0
        for j in range (30):
    		pub.publish(desired_velocity)
    		rate.sleep()
        desired_velocity.linear.x = 0
        desired_velocity.angular.z = 0.245
        for z in range (65):
    		pub.publish(desired_velocity)
    		rate.sleep()

if __name__ == "__main__":
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass
