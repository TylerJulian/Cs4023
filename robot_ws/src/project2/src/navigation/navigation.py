import math

import rospy

from movement.movement_commands import MovementCommands


class Navigation:

    # distance to a goal in which robot calls it success
    SUCCESS_DISTANCE = 1
    DEBUG = True

    def __init__(self, dispatcher):
        self.__dispatcher = dispatcher
        task_stack = []
        while dispatcher.task_planner.is_next_task_exists():
            # Always check out on movement status
            if rospy.get_param("HALT"):
                return

            while rospy.get_param("WAIT"):
                # Wait while the WAIT resolves
                pass

            # Get next point
            if len(task_stack) == 0:
                # If the tasks stack is empty, get one from the planner
                next_task = self.__dispatcher.task_planner.get_next_task()
                task_stack.append(next_task)
                print("new task acquired: " + str(next_task))

            current_coordinates = self.__dispatcher.get_current_location()[0]
            if self.__calculate_distance(current_coordinates, task_stack[len(task_stack)-1]) <= Navigation.SUCCESS_DISTANCE:
                print("Success reaching the point")
                task_stack.pop(len(task_stack)-1)
            else:
                # TODO: Observe avoidance interferences
                # Amend Avoidance to stop trying to turn the robot, rather notify the navigation the angle and distance
                # of the obstacle
                self.move_to_the_point(task_stack[len(task_stack)-1])
                print(self.__dispatcher.get_current_location()[0])

    def move_to_the_point(self, point):
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
        '''
        if abs(g_angle - current_angle) <= 180:
            angle = g_angle-current_angle
            if angle < 0:
                clockwise = True
                angle = abs(angle)
        else:
            angle = g_angle + (360 - current_angle)
            clockwise = True
        '''
        angle = g_angle - current_angle
        if angle < 0:
            clockwise = True
            angle = abs(angle)

        if Navigation.DEBUG:
            print("Next Move: ")
            print("Angle Between points: " + str(g_angle))
            print("Robot Angle change: " + str(angle))
            print("Distance: " + str(distance))
            print("Current angle: " + str(current_angle))

        # turn the robot
        MovementCommands.turn_robot(angle, clockwise)

        # move that robot
        MovementCommands.move_robot(distance)



    @staticmethod
    def __calculate_distance(pos1, pos2):
        return math.sqrt((pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2)
