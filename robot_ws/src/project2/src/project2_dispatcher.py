import rospy

from movement.bumper_control import BumperControl
from user_interface.plan_input import PlanInput
from movement.collision_avoidance.collision_avoidance import Avoidance


class Dispatcher:

    def __init__(self):
        # Set some parameters
        rospy.set_param("HALT", False)
        rospy.set_param("WAIT", False)
        # dispatch bumper control
        self.bumper_control = BumperControl(dispatcher=self)
        self.avoidance = Avoidance(dispatcher=self)
        while True:
            u_in = PlanInput.initial_user_prompt()
            if u_in == 1:
                l = PlanInput.get_coordinates_from_user()
                break
            elif u_in == 2:
                pass
                break
            else:
                print("Wrong option")
        self.kill()

    def kill(self):
        self.bumper_control.kill()
        self.avoidance.kill()

