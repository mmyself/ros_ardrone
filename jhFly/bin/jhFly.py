#!/usr/bin/env python
import roslib; roslib.load_manifest('jhFly')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

import sys, select, termios, tty


if __name__=="__main__":

	pub = rospy.Publisher('cmd_vel', Twist)
	land_pub = rospy.Publisher('/ardrone/land', Empty)
	reset_pub = rospy.Publisher('/ardrone/reset', Empty)
	takeoff_pub = rospy.Publisher('/ardrone/takeoff', Empty)

	rospy.init_node('jhFly')

	try:
		while(True):
			key = getKey()

			# takeoff and landing
			if key == 'l':
				land_pub.publish(Empty())
			if key == 'r':
				reset_pub.publish(Empty())
			if key == 't':
				takeoff_pub.publish(Empty())

                        if ord(key) == 27:
                            key = getKey()
                            key = getKey()
			twist = Twist()
			if ord(key) in move_bindings.keys():
                                key = ord(key)
			if key in move_bindings.keys():
				(lin_ang, xyz, speed) = move_bindings[key]
				setattr(getattr(twist, lin_ang), xyz, speed)
			else:
				if (key == '\x03'):
					break
			#print twist
			#print
			pub.publish(twist)

	except Exception as e:
		print e
		print repr(e)

	finally:
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub.publish(twist)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
