#!/usr/bin/env python
import roslib; roslib.load_manifest('jhFly')
import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import sys, select, termios, tty

def odomCallback(odom_msg):
	twist = Twist()
	twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
	
	print msg.pose.pose

	if(msg.pose.z < 0.9):
		twist.linear.z = 1		
	
	pub.publish(twist)


if __name__=="__main__":
	
	rospy.init_node('jhFly')

	pub = rospy.Publisher('cmd_vel', Twist)
	rospy.Subscriber("odom", Odometry, odomCallback)
	rospy.spin()
