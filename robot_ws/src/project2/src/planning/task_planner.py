import rospy

from user_interface.plan_input import PlanInput

class TaskPlanner:

    PLAN_EXIST = False
    TASK_LIST = None
    PLAN_LIST = None
    curr_location = (3.281, 2.5)


    def __init__(self,location):
        self.timer = rospy.Timer(rospy.Duration(secs=1), self.control())
        self.curr_location = location
    #def control(self, timer_event):
    def control(self):
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
        list_start = []
        list_end = []
        #EXAMPLE_TASKS = [((2, 2.5), (12.5, 2.5)), ((12.5, 12.5), (5, 12.5))]
        for x in self.TASK_LIST:
            #print(x)
            list_start.append(x[0])
            list_end.append(x[1])
        #print(list_start)
        list_start.sort(key = self.distance)
        list_end.sort(key = self.distance)
        #print(list_end)
        self.TASK_LIST = list_start + list_end
        print(self.TASK_LIST)

    def distance(self, pos):
        #print(pos)
        #print(self.curr_location[0])
        return (pos[0]-self.curr_location[0])**2 + (pos[1] - self.curr_location[1])**2
