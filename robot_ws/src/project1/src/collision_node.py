#!/usr/bin/env python

import rospy
from std_msgs.msg import String #generic messages
from geometry_msgs.msg import Twist, Vector3 #velocity
from sensor_msgs.msg import LaserScan #fake laser scan created by camera point cloud 

from laser_work import laser_work as lw


laser_info = None

def laser_read(lazer_msg):
	global laser_info
	laser_info = lazer_msg
	

def print_ranges(timer_event):
	lw.detect_obstacles(laser_info)

def main():

    # init collision node
    rospy.init_node('collision_node', anonymous=True)
    rospy.loginfo("collision_node started.")
	
	# subscribe to the laser 
    rospy.Subscriber('/scan', LaserScan, laser_read)

    rospy.Timer(rospy.Duration(2), print_ranges)

	# keep it going 
    rospy.spin()

if __name__ == '__main__':
	try:
	     main()
	except rospy.ROSInterruptException:
		pass
