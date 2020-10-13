import rospy
from movement.bumper_control import BumperControl
from user_interface.plan_input import PlanInput


class Dispatcher:

    HALT = False

    def __init__(self):
        # dispatch bumper control
        BumperControl(self)
        PlanInput.get_coordinates_from_user()



