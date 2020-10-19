import math

import rospy

from movement.movement_commands import MovementCommands


class Navigation:

    # distance to a goal in which robot calls it success
    SUCCESS_DISTANCE = 1
    DEBUG = False

    def __init__(self, dispatcher):
        self.__dispatcher = dispatcher

        task_stack = []  # A task stack in which robot uses to add and remove points to go.

        # some variables for other stat info
        total_number_of_points = 0
        number_of_successful_points = 0
        number_of_failed_points = 0
        failed_points_list = []

        '''
        This part is combines Navigation and Execution monitoring. 
        
        '''
        while dispatcher.task_planner.is_next_task_exists():
            # Always check out on movement status
            if rospy.get_param("HALT"):
                return

            while rospy.get_param("WAIT"):
                # Wait while the WAIT resolves
                pass

            if rospy.get_param("HOLD") and len(task_stack) > 0:
                # Investigate the avoidance issues
                obstacle_data = self.__dispatcher.avoidance.get_obstacle_info()
                if obstacle_data is not None:
                    print(task_stack)
                    divergence = self.create_avoidance_point(obstacle_data, task_stack[len(task_stack)-1])
                    if divergence is None:
                        # Cannot find a better way to reach the point; give up
                        print("Failed to reach the next point!")
                        failed_points_list.append(task_stack[0])
                        task_stack = []
                        rospy.set_param("HOLD", False)
                        number_of_failed_points += 1
                    else:
                        # add the divergence to the task stack
                        total_number_of_points += 1
                        task_stack.append(divergence)
                        rospy.set_param("HOLD", False)

            # Get next point
            if len(task_stack) == 0:
                # If the tasks stack is empty, get one from the planner
                next_task = self.__dispatcher.task_planner.get_next_task()
                total_number_of_points += 1
                task_stack.append(next_task)
                print("new task acquired: " + str(next_task))

            current_coordinates = self.__dispatcher.get_current_location()[0]
            if self.__calculate_distance(current_coordinates, task_stack[len(task_stack)-1]) <= Navigation.SUCCESS_DISTANCE:
                # Successful on reaching current goal
                number_of_successful_points += 1
                task_stack.pop(len(task_stack)-1)
            else:
                # move to next point on the stack
                self.move_to_the_point(task_stack[len(task_stack)-1])

        # all the points are tried to be reached,
        print("\n\n")
        print("-------------------------")
        print("Finished Navigating, Here are the stats: ")
        print("Total number of points attempted to navigate (this includes divergence points as well): " +
              str(total_number_of_points))
        print("Number of points successfully reached: " + str(number_of_successful_points))
        print("Number of failed points: " + str(number_of_failed_points))
        if number_of_failed_points > 0:
            print("  List of failed points: ")
            print("  " + str(failed_points_list))

        print("-------------------------")
        print("\n\n")

    def create_avoidance_point(self, obstacles_data, current_goal):
        """
        Tries to create a midpoint to avoid the obstacle while going toward the current goal
        @param obstacles_data:
        @param current_goal:
        @return: a new mid point
        """
        current_location = self.__dispatcher.get_current_location()[0]
        current_angle = self.__dispatcher.get_current_location()[1]

        if obstacles_data["direction"] == 'L' or obstacles_data["direction"] == 'R':
            # create a new point on the right or left
            angle = obstacles_data["angle"]
            divergence_vector = Navigation.calculate_divergence_vector(current_angle + angle, 2)
            new_point = tuple([current_location[0] + divergence_vector[0], current_location[1] + divergence_vector[1]])

            # check if new point will increase the path
            if self.examine_divergence(new_point, current_goal):
                return new_point
            else:
                return None

        elif obstacles_data['direction'] == 'C':
            # create points both on left and right
            divergence_vector_r = Navigation.calculate_divergence_vector(current_angle + 90, 2)
            divergence_vector_l = Navigation.calculate_divergence_vector(current_angle - 90, 2)

            new_point_r = tuple([current_location[0] + divergence_vector_r[0], current_location[1] + divergence_vector_r[1]])
            new_point_l = tuple([current_location[0] + divergence_vector_l[0], current_location[1] + divergence_vector_l[1]])

            # check if new points wont increase the path
            if self.examine_divergence(new_point_r, current_goal):
                return new_point_r
            elif self.examine_divergence(new_point_l, current_goal):
                return new_point_l
            else:
                # None of new point pass the examination, fail
                return None

    def move_to_the_point(self, point):
        """
        Moves the robot to the given coordinate
        @param point: coordinate to move
        @return: None
        """
        current_location_info = self.__dispatcher.get_current_location()
        current_coordinates = current_location_info[0]
        current_angle = current_location_info[1]
        goal = point

        # At this point, we need two things:
        # 1- Distance
        distance = Navigation.__calculate_distance(current_coordinates, goal)

        # 2- Angle
        g_angle = math.degrees(math.atan2(goal[1]-current_coordinates[1], goal[0]-current_coordinates[0]))

        # find out which side would be closer to turn
        clockwise = False

        if abs(g_angle - current_angle) <= 180:
            angle = g_angle-current_angle
            if angle < 0:
                clockwise = True
                angle = abs(angle)
        else:
            angle = 360 - abs(g_angle - current_angle)
            clockwise = True

        if Navigation.DEBUG:
            print("Next Move: ")
            print("Angle Between points: " + str(g_angle))
            print("Robot Angle change: " + str(angle))
            print("Distance: " + str(distance))
            print("Current angle: " + str(current_angle))

        # turn the robot to correct direction
        MovementCommands.turn_robot(angle, clockwise)

        # move that robot to the point
        MovementCommands.move_robot(distance)

    @staticmethod
    def calculate_divergence_vector(angle, distance):
        """
        Calculates a vector by a given angle and distance
        @param angle:
        @param distance:
        @return: new point (x,y)
        """
        x = distance * math.cos(math.radians(angle))
        y = distance * math.sin(math.radians(angle))
        return tuple([x, y])

    def examine_divergence(self, diverge_point, goal):
        """
        Checks if a diverged point will increase the distance to the goal
        @param diverge_point:
        @param goal:
        @return: True if not, False if new point will increase the distance
        """
        current_location = self.__dispatcher.get_current_location()[0]
        current_distance = Navigation.__calculate_distance(current_location, goal)
        diverge_distance = Navigation.__calculate_distance(diverge_point, goal)

        if diverge_distance <= current_distance:
            return True
        else:
            return False


    @staticmethod
    def __calculate_distance(pos1, pos2):
        """
        Calculates the distance between two point
        @param pos1: Point 1
        @param pos2: Point 2
        @return: distance: float
        """
        return math.sqrt((pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2)
