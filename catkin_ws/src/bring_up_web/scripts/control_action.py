#!/usr/bin/env python3

""" CALCULATE CONTROL ACTIONS (CRUISE SPEED & STEERING ANGLE) """

# LIBRARIES
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray


# GLOBAL VARIABLES
left_lane = [0.0, 0.0]
right_lane = [0.0, 0.0]

# LEFT LANE CALLBACK
def callback_left_lane(msg):
    global left_lane
    left_lane = list(msg.data)                                                          # TUPLE TO LIST

# RIGHT LANE CALLBACK
def callback_right_lane(msg):
    global right_lane
    right_lane = list(msg.data)                                                         # TUPLE TO LIST


# CALCULATE STEERING ANGLE ACCORDING TO LEFT LINE OR RIGHT LINE
def calculate_steering_angle(left_line, right_line, side):
    kd = 0.003                                                                          # CONSTANT FOR DISTANCE ERROR
    ka = 0.01                                                                           # CONSTANT FOR ANGLE ERROR
    
    detected_distance, detected_angle = [0.0, 0.0]                                      # INITIAL STATE FOR DETECTED MEASURES
    goal_distance, goal_angle = [0.0, 0.0]                                              # INTIAL STATE FOR GOAL MEASURES
    steering = 0.0                                                                      # INITIAL STATE FOR STEERING

    if side:                                                                            # IF ONLY THERE ARE LEFT LINES
        detected_distance, detected_angle = left_line                                   # DETECTED MEASURES FOR LEFT LINES
        goal_distance, goal_angle = [190.8952592391964, 0.7042371269405747]            # GOAL MEASURES FOR LEFT LINES

    else:                                                                               # IF ONLY THERE ARE RIGHT LINES
        detected_distance, detected_angle = right_line                                  # DETECTED MEASURES FOR RIGHT LINES
        goal_distance, goal_angle = [197.85095400325974, 0.6829543098903239]             # GOAL MEASURES FOR RIGHT LINES

    ed = goal_distance - detected_distance                                              # CALCULATE DISTANCE ERROR
    ea = goal_angle - detected_angle                                                    # CALCULATE ANGLE ERROR

    if ed == 0.0 or ea == 0.0:                                                          # THE CAR IS ALIGNED 
        steering = 0.0
    elif side:                                                                          # THE CAR IS NOT ALIGNED
        steering = (kd * ed) + (ka * ea)                                                # CALCULATE STEERING ACCORDING TO LEFT LINES
        print('ONLY LEFT LINE: ', steering)
    else:                                                                               # THE CAR IS NOT ALIGNED
        steering = (-1) * ((kd * ed) + (ka * ea))                                       # CALCULATE STEERING ACCORDING TO RIGHT LINES
        print('ONLY RIGHT LINE: ', steering)

    return steering                                                                     # RETURN THE CORRESPOND STEERING


def calculate_steering_angle_avg(left_line, right_line):
    kd = 0.0001
    ka = 0.01

    detec_dist_left, detec_angle_left = left_line                                       # DETECTED MEASURES FOR LEFT LINES
    detec_dist_right, detec_angle_right = right_line                                    # DETECTED MEASURES FOR RIGHT LINES

    avg_detec_dist = (detec_dist_left + detec_dist_right)/2                             # AVG OF DETECTED DISTANCE 
    avg_detec_angle = (detec_angle_left + detec_angle_right)/ 2                         # AVG OF DETECTED ANGLE 

    # CHECK IN FIRST FRAME
    goal_dist_left, goal_angle_left = [183.5980664386202, 0.7932720164215489]           # GOAL MEASURES FOR LEFT LINES
    goal_dist_right, goal_angle_right = [197.41390528531673, 0.6342421141311613]        # GOAL MEASURES FOR RIGHT LINES

    avg_goal_dist = (goal_dist_left + goal_dist_right)/2                                # AVG OF GOAL DISTANCE
    avg_goal_angle = (goal_angle_left + goal_angle_right)/2                             # AVG OF GOAL ANGLE

    ed = avg_goal_dist - avg_detec_dist                                                 # CALCULATE DISTANCE ERROR
    ea = avg_goal_angle - avg_detec_angle                                               # CALCULATE ANGLE ERROR

    if ed == 0.0 or ea == 0.0:                                                          # THE CAR IS ALIGNED
        steering = 0.0                                                                  
    else:
        steering = (kd * ed) + (ka * ea)                                                # CALCULATE STEERING ACCORDING THE AVGS

    print('AVG STEERING: ', steering)

    return steering                                                                     # RETURN THE CORRESPOND ANGLE


# MAIN FUNCTION
def main():

    global left_lane
    global right_lane

    cruise_speed = 0.0
    steering_angle = 0.0 

    # INIT NODE
    print('Control Action node ...')
    rospy.init_node('control_action')
    rate = rospy.Rate(10)  # 10Hz

    # SUBSCRIBERS
    rospy.Subscriber('/left_lane', Float64MultiArray, callback_left_lane)
    rospy.Subscriber('/right_lane', Float64MultiArray, callback_right_lane)

    # PUBLISHERS
    pub_speed = rospy.Publisher('/goal_cruise_speed', Float64, queue_size=10)
    pub_angle = rospy.Publisher('/goal_steering_angle', Float64, queue_size=10)


    while not rospy.is_shutdown():
        
        # LAW CONTROL
        if (left_lane[0] == 0.0 and left_lane[1] == 0.0) and (right_lane[0] == 0.0 and right_lane[1] == 0.0):           # NO LINES DETECTED
            cruise_speed = 0.0
            steering_angle = 0.0
        elif (left_lane[0] != 0.0 and left_lane[1] != 0.0) and (right_lane[0] != 0.0 and right_lane[1] != 0.0):         # BOTH LINES DETECTED
            cruise_speed = 30.0
            steering_angle = calculate_steering_angle_avg(left_lane, right_lane)
        elif (left_lane[0] != 0.0 and left_lane[1] != 0.0) and (right_lane[0] == 0 or right_lane[1] == 0.0):            # LEFT LINES DETECTED
            cruise_speed = 20.0
            steering_angle = calculate_steering_angle(left_lane, right_lane, True)
        elif (left_lane[0] == 0.0 or left_lane[1] == 0.0) and (right_lane[0] != 0 and right_lane[1] != 0.0):            # RIGHT LINES DETECTED
            cruise_speed = 20.0
            steering_angle = calculate_steering_angle(left_lane, right_lane, False)

        pub_speed.publish(cruise_speed)
        pub_angle.publish(steering_angle)
        rate.sleep()
        pass
    

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass


