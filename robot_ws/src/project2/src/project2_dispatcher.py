import math
import subprocess

import rospy
from nav_msgs.msg import Odometry

from navigation.movement.movement_commands import MovementCommands
from navigation.movement.bumper_control import BumperControl
from navigation.movement.collision_avoidance.collision_avoidance import Avoidance
from planning.task_planner import TaskPlanner
from navigation.navigation import Navigation

class Dispatcher:
    """
    The dispatcher, dispatches different services and contains information about robot
    such as its approximated location compared to the corner of the room
    """

    __GLOBAL_OFFSET = (3.281, 2.5)
    __current_location = (0, 0)
    __angle = 0
    __dist_from_task = 0
    __last_dist = 0

    def __init__(self):
        # Set some parameters
        rospy.set_param("HALT", False)
        rospy.set_param("WAIT", False)
        rospy.set_param("HOLD", False)
        # Subscribe to odometry
        rospy.Subscriber('odom', Odometry, self.__parse_current_location)

        # dispatch bumper control
        self.bumper_control = BumperControl(dispatcher=self)
        self.avoidance = Avoidance(dispatcher=self, is_navigation=True)

        # wait until laser is ready
        while not self.avoidance.is_laser_loaded():
            pass

        # clear the stdout and wait
        self.shell = subprocess.Popen("clear", shell=True)
        self.shell.wait()

        print("Welcome")

        while not rospy.is_shutdown():

            # dispatch task planner
            self.task_planner = TaskPlanner(dispatcher=self)

            # dispatch navigation
            self.navigation = Navigation(dispatcher=self)

    def __parse_current_location(self, data):
        # retrieve positional information from data and convert to feet and degree
        c_x = Dispatcher.to_feet(data.pose.pose.position.x)
        c_y = Dispatcher.to_feet(data.pose.pose.position.y)

        # calculate the angle from Quaternion to Euler angles conversion
        # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        x = data.pose.pose.orientation.x
        y = data.pose.pose.orientation.y
        z = data.pose.pose.orientation.z
        w = data.pose.pose.orientation.w

        c_a = math.degrees(math.atan2(2*((w*z)+(x*y)), 1-2*((y**2) + (z**2))))

        # Add global offset to the locational data
        c_x += Dispatcher.__GLOBAL_OFFSET[0]
        c_y += Dispatcher.__GLOBAL_OFFSET[1]

        # Set variables
        self.__current_location = (c_x, c_y)
        self.__angle = c_a

    def get_current_location(self):
        """
        gets the current location information with current angle inform of
        ((x,y),angle)
        @return:((x,y),angle) : tuple
        """
        return tuple([self.__current_location, self.__angle])


    @staticmethod
    def to_feet(meter):
        return meter * 3.281

    def kill(self):
        self.bumper_control.kill()
        self.avoidance.kill()

