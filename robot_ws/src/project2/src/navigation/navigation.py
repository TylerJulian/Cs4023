import math

from movement.movement_commands import MovementCommands


class Navigation:
    def __init__(self, dispatcher):
        self.__dispatcher = dispatcher
        self.move_to_next_point()

    def move_to_next_point(self):
        current_location_info = self.__dispatcher.get_current_location()
        current_coordinates = current_location_info[0]
        current_angle = current_location_info[1]
        goal = self.__dispatcher.task_planner.get_next_task()

        # At this point, we need two things:
        # 1- Distance
        distance = Navigation.__calculate_distance(current_coordinates, goal)

        # 2- Angle
        g_angle = math.degrees(math.atan2(goal[1]-current_coordinates[1], goal[0]-current_coordinates[1]))

        # find out which side would be closer to turn
        clockwise = False
        if abs(g_angle - current_angle) <= 180:
            angle = g_angle-current_angle
        else:
            angle = g_angle + (360 - current_angle)
            clockwise = True

        # turn the robot
        MovementCommands.turn_robot(angle, clockwise)

        # move that robot
        MovementCommands.move_robot(distance)


        print("Angle: " + str(angle))


    @staticmethod
    def __calculate_distance(pos1, pos2):
        return math.sqrt((pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2)
