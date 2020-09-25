#!/usr/bin/env python

import rospy
from std_msgs.msg import String #generic messages
from geometry_msgs.msg import Twist, Vector3 #velocity
from sensor_msgs.msg import LaserScan #fake laser scan created by camera point cloud 

from laser_work import laser_work as lw
import math

# Speed which the robot with turn (degree/second)
ANGULAR_SPEED = 5

# laser message (data from laser sensor sub)
laser_info = None

# publisher
pub = None


def laser_read(laser_msg):
	global laser_info
	laser_info = laser_msg


def turn_robot(angle,clockwise):
	direction_command = Twist()
	direction_command.linear = Vector3(0,0,0)
	direction_command.angular = Vector3(0,0,)

def avoid_collision(timer_event):
	global laser_info
	try:
		obstcl_exist, obstcl_dir, obstcl_angle, obstcl_distance = lw.detect_obstacles(laser_info)
	except AttributeError:
		print("Laser not loaded yet")
		return

	if obstcl_exist == False:
		return
	else:
		
		if obstcl_dir == lw.CENTER:
			print("Obstacle in center")
			# turn back
		elif obstcl_dir == lw.RIGHT:
			print("Obstacle in right")
			# turn left
		elif obstcl_dir == lw.LEFT:
			print("Obstacle in left")
			# turn right

def main():

    # init collision node
    rospy.init_node('collision_node', anonymous=True)
    rospy.loginfo("collision_node started.")
	
	# subscribe to the laser 
    rospy.Subscriber('/scan', LaserScan, laser_read)

    global pub
    pub = rospy.Publisher("/mobile_base/commands/velocity",Twist, queue_size=1)

    rospy.Timer(rospy.Duration(.2), avoid_collision)

	# keep it going 
    rospy.spin()

if __name__ == '__main__':
	try:
	     main()
	except rospy.ROSInterruptException:
		pass
