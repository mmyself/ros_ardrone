#!/usr/bin/env python
import roslib; roslib.load_manifest('drone_teleop')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
up/down:       move forward/backward
left/right:    move left/right
w/s:           increase/decrease altitude
a/d:           turn left/right
t/l:           takeoff/land
r:             reset (toggle emergency state)
anything else: stop

please don't have caps lock on.
CTRL+c to quit
"""

move_bindings = {
		68:('linear', 'y', 0.1), #left
		67:('linear', 'y', -0.1), #right
		65:('linear', 'x', 0.1), #forward
		66:('linear', 'x', -0.1), #back
		'w':('linear', 'z', 0.1),
		's':('linear', 'z', -0.1),
		'a':('angular', 'z', 1),
		'd':('angular', 'z', -1),
	       }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	print msg
	
	pub = rospy.Publisher('cmd_vel', Twist)
	land_pub = rospy.Publisher('/ardrone/land', Empty)
	reset_pub = rospy.Publisher('/ardrone/reset', Empty)
	takeoff_pub = rospy.Publisher('/ardrone/takeoff', Empty)

	rospy.init_node('drone_teleop')

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


