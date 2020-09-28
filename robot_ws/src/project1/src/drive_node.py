#!/usr/bin/env python

import rospy
from std_msgs.msg import String #generic messages
from geometry_msgs.msg import Twist, Vector3 #velocity
from kobuki_msgs.msg import BumperEvent # we need this for bumper collision

#main publisher
pub = None
#most recent keyboard command
keyboard_command = Twist()
#most recent bumper_state
is_bumped = False
# most recent planned command
path_plan_command = Twist()

def get_keyboard_command(key_msg):
	global keyboard_command
	keyboard_command = key_msg
	# force other nodes to stop moving and wait when there is a keyboard
	if not is_twist_empty(keyboard_command):
		rospy.set_param("KEY", True)
		
	# print(keyboard_command)

def bumper_state(bumper_state):
	global is_bumped
	is_bumped = bumper_state.state == 1 or is_bumped
	# print(is_bumped)

def publish_command(timer_event):
	#This will be called if the bumper is pressed down
	if is_bumped == 1:
		# Set a Halt parameter to true so other nodes stop trying to move 
		rospy.set_param("HALT", True)

		stop_movement_command = Twist()
		stop_movement_command.linear = Vector3(0,0,0)
		stop_movement_command.angular = Vector3(0,0,0)
		pub.publish(stop_movement_command)

		

		#print("bumped")
		return

	#Checks to see if there is a keyboard command or not. If there is no keyboard command then the path_planning_command is published
	if is_twist_empty(keyboard_command):
		pub.publish(path_plan_command)
	else:
		pub.publish(keyboard_command)
		rospy.set_param("KEY", False)
		# clear the key wait restriction
		

#check if the twist message is zero
def is_twist_empty(keyboard_command):
	state_linear = (keyboard_command.linear.x == keyboard_command.linear.y == keyboard_command.linear.z == 0)
	state_angular = (keyboard_command.angular.x == keyboard_command.angular.y == keyboard_command.angular.z == 0)
	return (state_linear == state_angular)

def main():
	#main publisher
	global pub

	#init node
	rospy.init_node("drive_node", anonymous=True)
	#global publisher
	pub = rospy.Publisher("/mobile_base/commands/velocity",Twist, queue_size=1)

	#subscribe to keyboard
	rospy.Subscriber("/project1/keyboard_input", Twist, get_keyboard_command)
	#subscribe to bumper
	rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, bumper_state)

	# Create a halt parameter, this will be used by other nodes to stop when true
	rospy.set_param("HALT", False)

	# Create a wait parameter, this will used to negotiate movement between nodes
	rospy.set_param("WAIT", False)

	# Create a KEY parameter which specifies that there is a keyboard entry, force other nodes stop
	rospy.set_param("KEY", False)

	#send command 5 times every second
	rospy.Timer(rospy.Duration(secs = .2), publish_command)
	#do it again
	rospy.spin()

if __name__ == '__main__':
	try:
	     main()
	except rospy.ROSInterruptException:
		pass
