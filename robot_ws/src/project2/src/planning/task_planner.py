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
# this function sorts the given list by the distance of the first coordinate pairs from the starting point
# The second point is immediately added after the firsts point.
# This makes it easy to remove the secondary task from the list.
# Here is what the example given points become. First line is the input list of points and the second list is the output
#[((2, 2.5), (12.5, 2.5)), ((12.5, 12.5), (5, 12.5)), ((2, 3), (9, 8))]
#[(2, 2.5), (12.5, 2.5), (2, 3), (9, 8), (12.5, 12.5), (5, 12.5)]

    def create_new_plan(self):
        print(self.TASK_LIST)
        self.TASK_LIST.sort(key = self.distance)
        temp_list = []
        for x in self.TASK_LIST:
            temp_list.append(x[0])
            temp_list.append(x[1])
        self.TASK_LIST = temp_list
        print(self.TASK_LIST)

    def distance(self, pos):
        #print(pos)
        #print(self.curr_location[0])
        return (pos[0][0]-self.curr_location[0])**2 + (pos[0][1] - self.curr_location[1])**2
