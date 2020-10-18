import math

import rospy

from user_interface.plan_input import PlanInput
from journey.journey import Journey, JourneyBook

class TaskPlanner:

    PLAN_EXIST = False  # True when there is a plan
    TASK_LIST = []  # List of points which robot have to reach
    PLANED_LIST = []  # List of the plans
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
        Creates a plan from the tasks.
        let have three tasks
        1 - (x1,y1) -> (x2, y2)
        2- (a1, b1) -> (a2, b2)
        3- (m1, n1) -> (m2, n2)

        The plan will be to move the closest starting point first, then either move to another closest starting point,
        or an ending point which went to it's starting point before. All following the closest path first.
        @return None
        """

        starting_points_group = list()
        ending_points_group = list()
        consideration_group = list()
        comparing_point = self.dispatcher.get_current_location()[0]

        for t in self.TASK_LIST:
            starting_points_group.append(t[0])
            ending_points_group.append(t[1])

        while len(starting_points_group) > 0 or len(consideration_group) > 0:

            if len(starting_points_group) == 0:
                # Case all group a is depleted meaning we only left with ending points
                i = TaskPlanner.get_min_distance_index(comparing_point, consideration_group)
                point = consideration_group.pop(i)
                self.PLANED_LIST.append(point)
                comparing_point = point

            else:
                i_1 = TaskPlanner.get_min_distance_index(comparing_point, starting_points_group)
                i_2 = TaskPlanner.get_min_distance_index(comparing_point, consideration_group)

                dist_1 = TaskPlanner.distance2(comparing_point, starting_points_group[i_1])
                dist_2 = 1000
                if i_2 >= 0:
                    dist_2 = TaskPlanner.distance2(comparing_point, consideration_group[i_2])

                if i_2 < 0 or dist_1 <= dist_2:

                    # take out one from starting points
                    start_point = starting_points_group.pop(i_1)
                    end_point = ending_points_group.pop(i_1)
                    print(start_point)
                    print(end_point)
                    consideration_group.append(end_point)
                    self.PLANED_LIST.append(start_point)
                    comparing_point = start_point

                else:
                    point = consideration_group.pop(i_2)
                    self.PLANED_LIST.append(point)
                    comparing_point = point

        self.TASK_QUEUE = self.PLANED_LIST
        self.PLAN_EXIST = True
        print(self.PLANED_LIST)






    @staticmethod
    def get_min_distance_index(point, p_list):
        """
        gets the index of closes point in p_list to given point
        @param point: the compare point
        @param p_list: list of points
        @return: index of closest point
        """
        closest_index = -1

        if len(p_list) == 0:
            return -1
        closest_distance = 1000.0
        for index in range(0, len(p_list)):
            distance = TaskPlanner.distance2(p_list[index], point)
            if distance <= closest_distance:
                closest_index = index
                closest_distance = distance

        return closest_index


    @staticmethod
    def distance2(a, b):
        """
        gets the distance between two point a and b
        @param a:
        @param b:
        @return: distance
        """

        return math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)


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
        return math.sqrt((pos[0][0]-self.curr_location()[0][1])**2 + (pos[0][1] - self.curr_location()[0][1])**2)

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
