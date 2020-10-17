import rospy

from user_interface.plan_input import PlanInput

class TaskPlanner:

    PLAN_EXIST = False  # True when there is a plan
    TASK_LIST = None  # List of points which robot have to reach
    PLAN_LIST = None  # List of the plans
    curr_location = (3.281, 2.5)  # default position. Gets immediately replaced by the actual position
    TASK_QUEUE = []  # A queue which is same as the task list, however the top will pop on every enquire

    def __init__(self, dispatcher):
        self.dispatcher = dispatcher
        # self.timer = rospy.Timer(rospy.Duration(secs=1), self.control)
        self.curr_location = self.dispatcher.get_current_location
        self.ask_user_for_plan()

    def control(self, timer_event):
        if not self.PLAN_EXIST:
            self.ask_user_for_plan()

    def ask_user_for_plan(self):
        """
        The user prompt
        @return: None
        """
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
        """
        this function sorts the given list by the distance of the first coordinate pairs from the starting point
        The second point is immediately added after the firsts point.
        This makes it easy to remove the secondary task from the list.
        Here is what the example given points become. First line is the input list of points and the second list is the output
        [((2, 2.5), (12.5, 2.5)), ((12.5, 12.5), (5, 12.5)), ((2, 3), (9, 8))]
        [(2, 2.5), (12.5, 2.5), (2, 3), (9, 8), (12.5, 12.5), (5, 12.5)]
        @return None
        """
        print(self.TASK_LIST)
        self.TASK_LIST.sort(key=self.distance)
        temp_list = []
        for x in self.TASK_LIST:
            temp_list.append(x[0])
            temp_list.append(x[1])
        self.TASK_LIST = temp_list
        self.TASK_QUEUE = self.TASK_LIST
        print(self.TASK_LIST)

    def distance(self, pos):
        """
        Calculates the distance from current location to the given position
        @param pos: given position
        @return: distance: float
        """
        #print(pos)
        #print(self.curr_location[0])
        # the location information is ((x,y),angle)
        # so x => location[0][0]
        # and y = location[0][1]
        return (pos[0][0]-self.curr_location()[0][1])**2 + (pos[0][1] - self.curr_location()[0][1])**2

    def get_next_task(self):
        """
        Pops the next task from queue and returns
        @return: (x,y) next task: tuple
        """
        return self.TASK_QUEUE.pop(0)

    def is_next_task_exists(self):
        """
        @return: True if there is a task to do: bool
        """
        return len(self.TASK_QUEUE) > 0
