import rospy
from geometry_msgs.msg import Twist, Vector3
from kobuki_msgs.msg import BumperEvent


class BumperControl:
    """
    Class Bumper Control

    Bumper control checks if a bumper collision registered and immediately stops the robot.
    """

    def __init__(self, dispatcher):
        self.is_bumped = False
        self.dispatcher = dispatcher
        self.publisher = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=1)
        rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, self.bumper_state)

        # ! Very important to register this publish command, therefore ros creates a thread
        rospy.Timer(rospy.Duration(secs=.2), self.publish_command)

    def bumper_state(self, bumper_state):
        self.is_bumped = bumper_state.state == 1 or self.is_bumped

    def publish_command(self, timer_event):
        # This will be called if the bumper is pressed down

        if self.is_bumped == 1:
            # Set a Halt parameter to true so other nodes stop trying to move
            self.dispatcher.HALT = True

            stop_movement_command = Twist()
            stop_movement_command.linear = Vector3(0, 0, 0)
            stop_movement_command.angular = Vector3(0, 0, 0)
            self.publisher.publish(stop_movement_command)

            # print("bumped")
            return

