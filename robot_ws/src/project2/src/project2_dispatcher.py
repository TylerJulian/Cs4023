from movement.bumper_control import BumperControl
from user_interface.plan_input import PlanInput
from movement.collision_avoidance.collision_avoidance import Avoidance


class Dispatcher:

    HALT = False

    def __init__(self):
        # dispatch bumper control
        BumperControl(self)
        Avoidance()
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




