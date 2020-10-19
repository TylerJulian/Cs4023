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

    __latest_collision_info = None
    __laser_loaded = False

    def __init__(self, dispatcher, is_navigation=False):
        self.dispatcher = dispatcher
        self._is_nav = is_navigation


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
            self.__laser_loaded = True
        except AttributeError:
            print("Laser not loaded yet")
            self.__laser_loaded = False
            return

        if not obstcl_exist:
            return
        else:
            # Register the obstacle information
            self.__latest_collision_info = {
                "direction": obstcl_dir,
                "angle": obstcl_angle,
                "distance": obstcl_distance
            }

            # set the WAIT parameter so other nodes stop trying to move the robot while turning
            rospy.set_param("WAIT", True)

            if self._is_nav:
                rospy.set_param("WAIT", False)
                rospy.set_param("HOLD", True)
                MovementCommands.stop_robot()
                return

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

    def is_laser_loaded(self):
        """
        @return: true if the laser is loaded and ready to work.
        """
        return self.__laser_loaded

    def get_obstacle_info(self):
        """
        @return: information about the latest obstacle :{"direction":<>, "angle":<>, "distance":<>}
        """
        return self.__latest_collision_info

    def kill(self):
        """
        Stops all the threads fired by this class
        @return: None
        """
        print("[Avoidance] killing")
        self.timer.shutdown()
