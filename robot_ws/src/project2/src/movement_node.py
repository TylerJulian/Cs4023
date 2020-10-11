#!/usr/bin/env python
import rospy
from rospy.timer import sleep
from std_msgs.msg import String #generic messages
from geometry_msgs.msg import Twist, Vector3 #velocity
import math
import random

pub = None # movement publisher

# The maximum limit of the turning in degree
MAX_TURNING_ANGLE = 15.0

# The speed for turning (degree/second)
ANGULAR_SPEED_ = 40

# linear movement speed (meter/second)
SPEED = 0.2

# distance to move before turning randomly meter (1 foot = 0.3048 meter)
DISTANCE_BEFORE_TURN = 0.3048

# A set of states 
FWD = "FORWARD"
TRN = "TURNING"

# the current action of the robot
current_state = FWD

# a semaphore to prevent extra move commands from stacking
is_currently_moving = False


def movement_is_ok():
    """
    checks both custom wait and halt parameters
    """
    return ((not rospy.get_param("HALT")) and (not rospy.get_param("WAIT"))) and (not rospy.get_param("KEY"))

def turn_robot(angle, clockwise):
	"""
	A function used to turn the robot, an imitation of code used in ros wiki 
	http://wiki.ros.org/turtlesim/Tutorials/Rotating%20Left%20and%20Right
	"""

	global pub
	global is_currently_moving

	# set rotation semaphore true
	is_currently_moving = True
	direction_command = Twist()
	
	# There is no linear movement
	direction_command.linear = Vector3(0,0,0)
    
	# convert angular speed to radian 
	angular_speed = math.radians(ANGULAR_SPEED_)
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
	while current_angle < relative_angle and movement_is_ok():
		pub.publish(direction_command)
		t_1 = rospy.Time.now().to_sec()
		current_angle = angular_speed*(t_1-t_0)

	# stop the rotation
	direction_command.angular = Vector3(0,0,0)
	pub.publish(direction_command);is_currently_moving = False



def move_robot(distance):
    """
    moves the robot straight on specified distance 
    """

    global pub
    global is_currently_moving
    is_currently_moving = True

    move_command = Twist()

    # no turning 
    move_command.angular = Vector3(0, 0, 0)

    # just moving forward
    move_command.linear = Vector3(SPEED, 0, 0)

    # moving forward for specified distance
    t_0 = rospy.Time.now().to_sec()
    current_distance = 0
    while current_distance < distance and movement_is_ok():
        pub.publish(move_command)
        t_1 = rospy.Time.now().to_sec()
        current_distance = SPEED*(t_1-t_0)
    
    move_command.linear = Vector3(0,0,0)
    pub.publish(move_command)
    is_currently_moving = False


def move(timer_event):
    """
    handles the movement
    """
    global current_state
    global is_currently_moving

    if rospy.get_param("KEY"):
        # wait if there is a keyboard command
        sleep(2)

    if is_currently_moving and (not movement_is_ok()):
        return

    if current_state == FWD:
        move_robot(DISTANCE_BEFORE_TURN)
        current_state = TRN
    else:
        # calculate a random angle to turn
        angle = random.randrange(-MAX_TURNING_ANGLE, MAX_TURNING_ANGLE)
        clockwise = angle < 0
        turn_robot(angle,clockwise)
        current_state = FWD
    


def main():

    # init collision node
    rospy.init_node('collision_node', anonymous=True)
    rospy.loginfo("collision_node started.")
	
	
    global pub
    pub = rospy.Publisher("/mobile_base/commands/velocity",Twist, queue_size=1)

    rospy.Timer(rospy.Duration(0.1),move)

	# keep it going 
    rospy.spin()

if __name__ == '__main__':
	try:
	     main()
	except rospy.ROSInterruptException:
		pass
