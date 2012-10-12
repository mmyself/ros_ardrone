#!/usr/bin/env python
import roslib; roslib.load_manifest('jhFly')
import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import sys, select, termios, tty

def odomCallback(odom_msg):
	
	twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
	
	passedTime = rospy.Time.now() - startTime;
	
	print odom_msg.pose.pose
	print passedTime

	if(passedTime > 2.0 and odom_msg.pose.pose.position.z < 0.9):
		twist.linear.z = 1

	if(passedTime > 5.0 and odom_msg.pose.pose.position.x < 0.4):
		twist.linear.x = 1

	if(passedTime > 8.0 and odom_msg.pose.pose.position.y < 0.4):
		twist.linear.y = 1

	if(passedTime > 11.0 and odom_msg.pose.pose.position.x > 0.1):
		twist.linear.x = -1

	if(passedTime > 13.0 and odom_msg.pose.pose.position.y > 0.1):
		twist.linear.y = -1

	pub.publish(twist)


if __name__=="__main__":
	
	rospy.init_node('jhFly')
	twist = Twist()
	startTime = rospy.Time.now()
	
	pub = rospy.Publisher('cmd_vel', Twist)
	rospy.Subscriber("odom", Odometry, odomCallback)
	rospy.spin()
