"""
Collision Avoidance Module
"""

import rospy
import random

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from laser_work import LaserWorks as lw

from .. movement_commands import MovementCommands


class Avoidance:
    """
    Avoidance class creates a thread that monitors robot's laser data and
    prevents hitting obstacles.
    """

    def __init__(self, dispatcher):
        self.dispatcher = dispatcher
        # Get the movement publisher


        # The laser info come from the sensor
        self.laser_info = None

        # subscribe to laser sensor
        rospy.Subscriber('/scan', LaserScan, self.laser_read)
        self.timer = rospy.Timer(rospy.Duration(secs=.2), self.avoid_collision)

    def laser_read(self, laser_msg):
        self.laser_info = laser_msg

    def avoid_collision(self, timer_event):
        """
        Uses laser_info to check if there is an object in front of the robot
        Symmetric object classified as an obstacle in center, and asymmetric objects
        are classified as an obstacle in the right or left.

        """

        # return if halt command initiated
        if rospy.get_param("HALT"):
            return

        try:
            # Detection
            obstcl_exist, obstcl_dir, obstcl_angle, obstcl_distance = lw.detect_obstacles(self.laser_info)
        except AttributeError:
            print("Laser not loaded yet")
            return

        if not obstcl_exist:
            return
        else:

            # set the WAIT parameter so other nodes stop trying to move the robot while turning
            rospy.set_param("WAIT", True)

            # introduce a small random angle to prevent corner deadlocks
            delta_theta = random.randrange(-3, 3)

            # decide what is obstacle and respond properly
            if obstcl_dir == lw.CENTER:
                # print("Obstacle in center")
                clockwise = obstcl_angle >= 0
                MovementCommands.turn_robot(abs(obstcl_angle) + 180 + delta_theta, clockwise)
            # turn back
            elif obstcl_dir == lw.RIGHT:
                # print("Obstacle in right")
                MovementCommands.turn_robot(90 - abs(obstcl_angle) + delta_theta, False)
            # turn left
            elif obstcl_dir == lw.LEFT:
                # print("Obstacle in left")
                MovementCommands.turn_robot(90 - abs(obstcl_angle) + delta_theta, True)
            # turn right

            # set the wait param back to false
            rospy.set_param("WAIT", False)

    def kill(self):
        """
        Stops all the threads fired by this class
        @return: None
        """
        print("[Avoidance] killing")
        self.timer.shutdown()
