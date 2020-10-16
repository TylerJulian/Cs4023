import rospy

from user_interface.plan_input import PlanInput

class TaksPlanner:

    PLAN_EXIST = False
    TASK_LIST = None
    PLAN_LIST = None
    curr_location = None

    def __init__(self,location):
        self.timer = rospy.Timer(rospy.Duration(secs=1), self.control())
        self.curr_location = location
    def control(self, timer_event):
        if not self.PLAN_EXIST:
            self.ask_user_for_plan()

    def ask_user_for_plan(self):
        while True:
            u_in = PlanInput.initial_user_prompt()
            if u_in == 1:
                self.TASK_LIST = PlanInput.get_coordinates_from_user()
                self.create_new_plan()
                break
            elif u_in == 2:
                self.TASK_LIST = PlanInput.EXAMPLE_TASKS
                self.create_new_plan()
                break
            elif u_in == -2:
                break
            else:
                print("Wrong option")

    def create_new_plan(self):
        print(self.TASK_LIST)

