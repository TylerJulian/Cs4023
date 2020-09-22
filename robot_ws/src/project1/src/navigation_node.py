#!/usr/bin/env python

import rospy
import time # not sure if we need time
from std_msgs.msg import String #generic messages
from geometry_msgs.msg import Twist, Vector3 #velocity?
from kobuki_msgs.msg import BumperEvent # we need this for bumper collision

#publisher
pub = None
#most recent keyboard command
keyboard_command = Twist()
#most recent bumper command
is_bumped = False
# most recent planned command
path_plan_command = Twist()



def main():
	print "hello"



#generic function to call the main function
if __name__ == '__main__':
	try:
	     main()
	except rospy.ROSInterruptException:
		pass
