#!/usr/bin/env python3

""" 
    CLASS TO COMPUTE CONTROL LAWS FOR KEEP DISTANCE & CRUISE BEHAVIORS

    CRUISE : 
        CONSTANT SPEED, STEERING ANGLE IS CALCULATED BY THE DETECTED LINES OF CURRENT LANE
    KEEP DISTANCE : 
        CONSTANT SPEED BUT MINIMUM, STEERING ANGLE IS CALCULATED BY THE DETECTED LINES OF CURRENT LANE

"""

class Control:
    
    # INIT OBJECTS
    def __init__( self ):
        self.cruise_speed = 0.0
        self.steering_angle = 0.0

    # CONTROL LAW FOR LANE TRACKING & KEEP DISTANCE
    def control_law(self, left_line, right_line, distance = 0):

        # NO LINES DETECTED
        if (left_line[0] == 0.0 and left_line[1] == 0.0) and (right_line[0] == 0.0 and right_line[1] == 0.0):
            self.cruise_speed = 0.0
            self.steering_angle = 0.0
        # BOTH LINES DETECTED
        elif (left_line[0] != 0.0 and left_line[1] != 0.0) and (right_line[0] != 0.0 and right_line[1] != 0.0):
            self.cruise_speed = 30.0 if distance == 0.0 else abs(distance/0.5)
            self.steering_angle = self.compute_steering_angle_avg(left_line, right_line)
        # LEFT LINES DETECTED
        elif (left_line[0] != 0.0 and left_line[1] != 0.0) and (right_line[0] == 0 or right_line[1] == 0.0):
            self.cruise_speed = 20.0 if distance == 0.0 else abs(distance/0.5)
            self.steering_angle = self.compute_steering_angle(left_line, right_line, True)
        # RIGHT LINES DETECTED
        elif (left_line[0] == 0.0 or left_line[1] == 0.0) and (right_line[0] != 0 and right_line[1] != 0.0):
            self.cruise_speed = 20.0 if distance == 0.0 else abs(distance/0.5)
            self.steering_angle = self.compute_steering_angle(left_line, right_line, False)

    # CONPUTE STEERING ANGLE BY SIDE
    def compute_steering_angle(self, left_line, right_line, side):
        kd = 0.003                                                                  # CONSTANT FOR DISTANCE ERROR
        ka = 0.01                                                                   # CONSTANT FOR ANGLE ERROR
    
        detected_distance, detected_angle = [0.0, 0.0]                              # INITIAL STATE FOR DETECTED MEASURES
        goal_distance, goal_angle = [0.0, 0.0]                                      # INTIAL STATE FOR GOAL MEASURES
        steering = 0.0                                                              # INITIAL STATE FOR STEERING

        if side:                                                                    # IF ONLY THERE ARE LEFT LINES
            detected_distance, detected_angle = left_line                           # DETECTED MEASURES FOR LEFT LINES
            goal_distance, goal_angle = [194.36306233438492, 0.6955929768432806]    # GOAL MEASURES FOR LEFT LINES

        else:                                                                       # IF ONLY THERE ARE RIGHT LINES
            detected_distance, detected_angle = right_line                          # DETECTED MEASURES FOR RIGHT LINES
            goal_distance, goal_angle = [190.46325104859469, 0.7064260556560561]    # GOAL MEASURES FOR RIGHT LINES

        ed = goal_distance - detected_distance                                      # CALCULATE DISTANCE ERROR
        ea = goal_angle - detected_angle                                            # CALCULATE ANGLE ERROR

        if ed == 0.0 or ea == 0.0:                                                  # THE CAR IS ALIGNED 
            steering = 0.0
        elif side:                                                                  # THE CAR IS NOT ALIGNED
            steering = (kd * ed) + (ka * ea)                                        # CALCULATE STEERING ACCORDING TO LEFT LINES
        else:                                                                       # THE CAR IS NOT ALIGNED
            steering = (-1) * ((kd * ed) + (ka * ea))                               # CALCULATE STEERING ACCORDING TO RIGHT LINES

        return steering                                                             # RETURN THE CORRESPOND STEERING

    # COMPUTE AVG STEERING ANGLE (BOTH LINES)
    def compute_steering_angle_avg(self, left_line, right_line):
        kd = 0.00005
        ka = 0.01

        detec_dist_left, detec_angle_left = left_line                                 # DETECTED MEASURES FOR LEFT LINES
        detec_dist_right, detec_angle_right = right_line                              # DETECTED MEASURES FOR RIGHT LINES

        avg_detec_dist = (detec_dist_left + detec_dist_right)/2                       # AVG OF DETECTED DISTANCE 
        avg_detec_angle = (detec_angle_left + detec_angle_right)/ 2                   # AVG OF DETECTED ANGLE 

        # CHECK IN FIRST FRAME
        goal_dist_left, goal_angle_left = [194.36306233438492, 0.6955929768432806]    # GOAL MEASURES FOR LEFT LINES
        goal_dist_right, goal_angle_right = [190.46325104859469, 0.7064260556560561]  # GOAL MEASURES FOR RIGHT LINES

        avg_goal_dist = (goal_dist_left + goal_dist_right)/2.0                        # AVG OF GOAL DISTANCE
        avg_goal_angle = (goal_angle_left + goal_angle_right)/2.0                     # AVG OF GOAL ANGLE

        ed = avg_goal_dist - avg_detec_dist                                           # CALCULATE DISTANCE ERROR
        ea = avg_goal_angle - avg_detec_angle                                         # CALCULATE ANGLE ERROR

        if ed == 0.0 or ea == 0.0:                                                    # THE CAR IS ALIGNED
            steering = 0.0                                                                  
        else:
            steering = (kd * ed) + (ka * ea)                                          # CALCULATE STEERING ACCORDING THE AVGS


        return steering                                                               # RETURN THE CORRESPOND ANGLE
        
        
        

