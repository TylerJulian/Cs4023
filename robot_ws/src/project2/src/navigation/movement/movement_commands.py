import math
import rospy
from geometry_msgs.msg import Twist, Vector3


class MovementCommands:

    def __init__(self):
        pass

    @staticmethod
    def turn_robot(angle, clockwise, speed=40):
        """
        A function used to turn the robot, an imitation of code used in ros wiki
        http://wiki.ros.org/turtlesim/Tutorials/Rotating%20Left%20and%20Right
        """
        # create the publisher
        pub = MovementCommands.__get_publisher()


        # set rotation semaphore true

        direction_command = Twist()

        # There is no linear movement
        direction_command.linear = Vector3(0, 0, 0)

        # convert angular speed to radian
        angular_speed = math.radians(speed)
        relative_angle = math.radians(angle)

        # Set rotation direction properly
        if clockwise:
            z_component = -abs(angular_speed)
        else:
            z_component = abs(angular_speed)

        direction_command.angular = Vector3(0, 0, z_component)
        # rotation

        # get initial time
        t_0 = rospy.Time.now().to_sec()
        current_angle = 0

        # start rotating
        while (current_angle < relative_angle) and (not rospy.get_param("HALT")):
            pub.publish(direction_command)
            t_1 = rospy.Time.now().to_sec()
            current_angle = angular_speed * (t_1 - t_0)

        # stop the rotation
        direction_command.angular = Vector3(0, 0, 0)
        pub.publish(direction_command)
        pub.unregister()

    @staticmethod
    def move_robot(distance, speed=0.2):
        """
        moves the robot straight on specified distance
        An imitation of code used in ros wiki
        http://wiki.ros.org/turtlesim/Tutorials/Rotating%20Left%20and%20Right

        @param distance distance to move in feet
        @param speed speed in which to move (default= 0.2)
        """
        # convert entered distance to meter
        distance = 0.3048 * distance

        pub = MovementCommands.__get_publisher()

        move_command = Twist()

        # no turning
        move_command.angular = Vector3(0, 0, 0)

        # just moving forward
        move_command.linear = Vector3(speed, 0, 0)

        # moving forward for specified distance
        t_0 = rospy.Time.now().to_sec()
        current_distance = 0
        while current_distance < distance and (not (rospy.get_param("HALT") or rospy.get_param("WAIT"))):
            pub.publish(move_command)
            t_1 = rospy.Time.now().to_sec()
            current_distance = speed * (t_1 - t_0)

        move_command.linear = Vector3(0, 0, 0)
        pub.publish(move_command)
        pub.unregister()


    @staticmethod
    def stop_robot():
        MovementCommands.move_robot(0)


    @staticmethod
    def __get_publisher():
        return rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=1)