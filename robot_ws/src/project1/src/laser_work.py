import math


class laser_work:
    """ 
    @version: 1 
    @Author: yashar


    laser work class provides series of tools to work with laser readings.
    """

    RIGHT = 'R'
    LEFT = 'L'
    CENTER = 'C'


    # 1 foot tolerance in meter 
    AVOIDANCE_DISTANCE = 0.3048

    # symmetric, asymmetric determination tolerance in angle 
    # Used to determine if the obstacle is symmetric or 
    ANGLE_TOLERANCE = 5.0


    @staticmethod
    def detect_obstacles(laser_info):
        """
        Detects when there is an obstacle in front of the robot. 

        @return: ObstacleExist:Bool, 
                 ObstacleDirection: laser_work.CENTER | laser_work.LEFT | laser_work.RIGHT | None,
                 ObstacleDirectionInAngle: float (Degree) | None
                 MinDistanceToObstacle: float (METER) | None
        """

        '''
        This is done by detecting all the nodes that are smaller in range than 
        AVOIDANCE_DISTANCE and figuring out if the object is somewhat symmetric
        or not. This is done by calculating the biggest angle in which the sensor
        reading is less than specified distance for each side from the middle,
        or angle zero. The difference between angle on two side, will roughly 
        indicated where the obstacle is oriented. 

        The laser rotation works in right-hand rule along the positive Z-axis, 
        meaning that the laser rotates and sample readings counterclockwise, right to 
        left
        '''
        # minimum reading angle, the right side of the robot
        min_angle = math.degrees(laser_info.angle_min)

        # maximum reading angle, the left side of the robot
        max_angle = math.degrees(laser_info.angle_max)

        # angle of increment for each reading
        increment = math.degrees(laser_info.angle_increment)

        # The array of distance readings in meter
        ranges = laser_info.ranges

        # total number of readings 
        total_reading_count = len(ranges)

        # middle or supposedly angle zero reading index
        middle_reading_index = math.floor(total_reading_count/2)

        # scan left and right sides 
        right_max_angle, min_right_distance = laser_work.scan_right_side(ranges,middle_reading_index,increment)
        left_max_angle, min_left_distance = laser_work.scan_left_side(ranges,middle_reading_index,increment)
        
        # find the minimum distance
        min_distance = min([min_left_distance,min_right_distance])

        # check if there is any obstacles in defined distance
        obstacle_exists = False
        if min_distance > laser_work.AVOIDANCE_DISTANCE:
            # No obstacles, just return 
            return False, None, None, None ### CONDITION 1 EXIT : No obstacles

        # find angle difference
        angle_delta = right_max_angle - left_max_angle

        obstacle_direction = None
        if abs(angle_delta) < laser_work.ANGLE_TOLERANCE:
            obstacle_direction = laser_work.CENTER
        else:
            if angle_delta > 0:
                # obstacle mainly is in left side
                obstacle_direction = laser_work.LEFT
            
            else:
                # obstacle mainly is in right side
                obstacle_direction = laser_work.RIGHT

        return True, obstacle_direction, angle_delta, min_distance 
        




        



    @staticmethod
    def scan_right_side(ranges, mri, increment):
        right_side_angle = 0
        min_distance = float('inf')
        for i in range(mri, -1, -1):
            if ranges[i] < laser_work.AVOIDANCE_DISTANCE:
                current_angle = (mri - i) * increment
                # update max angle
                if current_angle > right_side_angle:
                    right_side_angle = current_angle
                # update min_distance
                if ranges[i] < min_distance:
                    min_distance = ranges[i]
        
        return right_side_angle, min_distance
    
    @staticmethod
    def scan_left_side(ranges ,mri, increment):
        left_side_angle = 0
        min_distance = float('inf')
        for i in range(mri, len(ranges)):
            if ranges[i] < laser_work.AVOIDANCE_DISTANCE:
                current_angle = (i - mri) * increment
                # update max angle
                if current_angle > left_side_angle:
                    left_side_angle = current_angle
                # update min_distance
                if ranges[i] < min_distance:
                    min_distance = ranges[i]
        
        return left_side_angle, min_distance




        




