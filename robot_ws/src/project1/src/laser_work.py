import math


class laser_work:
    """ 
    @version: 1 
    @Author: yashar


    laser work class provides series of tools to work with laser readings
    note that the provided laser info are 


    """


    # 1 foot tollerance in meter 
    AVOIDANCE_DISTANCE = 0.3048


    @staticmethod
    def detect_obstacles(laser_info):
        min_angle = math.degrees(laser_info.angle_min)
        max_angle = math.degrees(laser_info.angle_max)
        increment = math.degrees(laser_info.angle_increment)
        print("min angle: " + str(min_angle))
        print("max_angle: " + str(max_angle))
        print("angle increment: " + str(increment))

