#!/usr/bin/env python

import rospy
from std_msgs.msg import String #generic messages
from geometry_msgs.msg import Twist, Vector3 #velocity
from sensor_msgs.msg import LaserScan #fake laser scan created by camera point cloud 

from laser_work import laser_work as lw
import math
import random

# Speed which the robot with turn (degree/second)
ANGULAR_SPEED = 40

# laser message (data from laser sensor sub)
laser_info = None

# publisher
pub = None

# a semaphore to prevent initiating two rotation command
is_rotating = False


def laser_read(laser_msg):
	global laser_info
	laser_info = laser_msg


def turn_robot(angle, clockwise):
	"""
	A function used to turn the robot, an imitation of code used in ros wiki 
	http://wiki.ros.org/turtlesim/Tutorials/Rotating%20Left%20and%20Right
	"""
	global is_rotating
	global pub
	

	# set rotation semaphore true
	is_rotating = True
	direction_command = Twist()
	
	# There is no linear movement
	direction_command.linear = Vector3(0,0,0)
    
	# convert angular speed to radian 
	angular_speed = math.radians(ANGULAR_SPEED)
	relative_angle = math.radians(angle)

	# Set rotation direction properly
	if clockwise:
		z_component = -abs(angular_speed)
	else:
		z_component = abs(angular_speed)
	
	direction_command.angular = Vector3(0,0,z_component)
	# rotation 

	# get initial time 
	t_0 = rospy.Time.now().to_sec()
	current_angle = 0

	# start rotating
	while (current_angle < relative_angle) and (not rospy.get_param("HALT")) :
		pub.publish(direction_command)
		t_1 = rospy.Time.now().to_sec()
		current_angle = angular_speed*(t_1-t_0)

	# stop the rotation
	direction_command.angular = Vector3(0,0,0)
	pub.publish(direction_command)

	# done rotating, set semaphore false
	is_rotating = False
	

	

def avoid_collision(timer_event):
	"""
	Uses laser_info to check if there is an object in front of the robot
	Symmetric object classified as an obstacle in center, and asymmetric objects 
	are classified as an obstacle in the right or left. 

	"""
	global laser_info
	global is_rotating

	# return if halt command initiated
	if rospy.get_param("HALT"):
		return

	try:
		# Detection 
		obstcl_exist, obstcl_dir, obstcl_angle, obstcl_distance = lw.detect_obstacles(laser_info)
	except AttributeError:
		print("Laser not loaded yet")
		return
    

	if obstcl_exist == False:
		return
	else:
		# if the robot already in rotation because of the previous avoidance command, just return
		if is_rotating:
			return

		# set the WAIT parameter so other nodes stop trying to move the robot while turning
		rospy.set_param("WAIT", True)

		# introduce a small random angle to prevent corner deadlocks 
		delta_theta = random.randrange(-3,3)

		# decide what is obstacle and respond properly
		if obstcl_dir == lw.CENTER:
			# print("Obstacle in center")
			clockwise = obstcl_angle >= 0
			turn_robot(abs(obstcl_angle) + 180 + delta_theta, clockwise)
			# turn back
		elif obstcl_dir == lw.RIGHT:
			# print("Obstacle in right")
			turn_robot(90 - abs(obstcl_angle) + delta_theta,False)
			# turn left
		elif obstcl_dir == lw.LEFT:
			# print("Obstacle in left")
			turn_robot(90 - abs(obstcl_angle) + delta_theta, True)
			# turn right

		# set the wait param back to false
		rospy.set_param("WAIT", False)

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
