import math

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from movement.bumper_control import BumperControl
from movement.movement_commands import MovementCommands
from movement.collision_avoidance.collision_avoidance import Avoidance
from planning.task_planner import TaskPlanner
class Dispatcher:
    """
    The dispatcher, dispatches different services and contains information about robot
    such as its approximated location compared to the corner of the room
    """

    __GLOBAL_OFFSET = (3.281, 2.5)
    __current_location = (0, 0)
    __angle = 0

    def __init__(self):
        # Set some parameters
        rospy.set_param("HALT", False)
        rospy.set_param("WAIT", False)
        # dispatch bumper control
        self.bumper_control = BumperControl(dispatcher=self)
        self.avoidance = Avoidance(dispatcher=self)
        #start up task_planner()
        self.TaskPlanner = TaskPlanner(location=self.__GLOBAL_OFFSET)
        # Subscribe to odometry
        rospy.Subscriber('odom', Odometry, self.__parse_current_location)

        rospy.Timer(rospy.Duration(secs=1), self.print_)

        self.publisher = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=1)
        rospy.Timer(rospy.Duration(secs=2), self.move_it)
	

    def print_(self, t):
        print(self.get_current_location())

    def move_it(self, t):
        MovementCommands.move_robot(self.publisher, 1)

    def __parse_current_location(self, data):
        # retrieve positional information from data and convert to feet and degree
        c_x = Dispatcher.to_feet(data.pose.pose.position.x)
        c_y = Dispatcher.to_feet(data.pose.pose.position.y)
        c_a = math.degrees(data.pose.pose.orientation.z)

        # Add global offset to the locational data
        c_x += Dispatcher.__GLOBAL_OFFSET[0]
        c_y += Dispatcher.__GLOBAL_OFFSET[1]

        # Set variables
        self.__current_location = (c_x, c_y)
        self.__angle = c_a

    def  get_current_location(self):
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

