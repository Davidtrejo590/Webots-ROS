#!/usr/bin/env python3

""" CALCULATE CONTROL ACTIONS (CRUISE SPEED & STEERING ANGLE) """

# LIBRARIES
from numpy.lib.function_base import angle
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray


# GLOBAL VARIABLES
left_lane = [0.0, 0.0]
right_lane = [0.0, 0.0]

# LEFT LANE CALLBACK
def callback_left_lane(msg):
    global left_lane
    left_lane = list(msg.data)

# RIGHT LANE CALLBACK
def callback_right_lane(msg):
    global right_lane
    right_lane = list(msg.data)

def calculate_steering_angle_right(lane):
    kd = 0.001
    ka = 1.0

    detected_distance, detected_angle = lane
    goal_distance, goal_angle = [148.9236381505636, 0.6301090523170392]
    ed = goal_distance - detected_distance
    ea = goal_angle - detected_angle

    if ed == 0.0 or ea == 0.0:
        steering = 0.0
    elif ea > 0.015:
        steering = -0.0174533
    else:
        steering = (kd * ed) + (ka * ea)

    print([ed, ea, steering])
    return steering


def calculate_steering_angle(lane):
    # goal_distance = 146.24038429927623
    # goal_angle =  0.646238328234549
    # # goal_distance = 125.73881660012552
    # # goal_angle = 0.6174452308546208
    # kd = 0.003 # 0.003
    # ka = 0.01       
    # steering = 0.0
    # detected_distace, detected_angle = lane
    # # print('Distance Detected: ', detected_distace)
    # # print('Angle Detected: ', detected_angle)

    # # if detected_angle < goal_angle:
    # #     steering = -0.0174533
    # # elif detected_angle > goal_angle:
    # #     steering = 0.0174533
    
    # ed = goal_distance - detected_distace
    # ea = goal_angle - detected_angle
    # # print([ed, ea])
    # steering = (ka * ed) + (ka * ea)
    # print([ed, ea, steering])

    kd = 0.003
    ka = 0.01

    detected_distance, detected_angle = lane
    goal_distance, goal_angle = [144.90341610879986, 0.6545707673233914]
    ed = goal_distance - detected_distance
    ea = goal_angle - detected_angle

    if ed == 0.0 or ea == 0.0:
        steering = 0.0
    else:
        steering = (kd * ed) + (ka * ea)

    print('Only Left Lane', steering)

    return steering

def calculate_steering_angle_avg(left_lane, right_lane):

    kd = 0.003
    ka = 1.0

    detec_dist_left, detec_angle_left = left_lane
    detec_dist_right, detec_angle_right = right_lane

    avg_dist_detec = (detec_dist_left + detec_dist_right)/2
    avg_angle_detec = (detec_angle_left + detec_angle_right)/ 2

    # CHECK IN FIRST FRAME
    goal_dist_left, goal_angle_left = [144.90341610879986, 0.6545707673233914]
    goal_dist_right, goal_angle_right = [148.9236381505636, 0.6301090523170392]

    avg_goal_dist = (goal_dist_left + goal_dist_right)/2
    avg_goal_angle = (goal_angle_left + goal_angle_right)/2

    ed = avg_goal_dist - avg_dist_detec
    ea = avg_goal_angle - avg_angle_detec

    if ed == 0.0 or ea == 0.0:
        steering = 0.0
    else:
        steering = (kd * ed) + (ka * ea)

    print('AVG STEERING')

    return steering




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

        if (left_lane[0] == 0.0 and left_lane[1] == 0.0) and (right_lane[0] == 0.0 and right_lane[1] == 0.0):
            cruise_speed = 0.0
            steering_angle = 0.0
        elif (left_lane[0] != 0.0 and left_lane[1] != 0.0) and (right_lane[0] != 0.0 and right_lane[1] != 0.0):
            cruise_speed = 10.0
            steering_angle = calculate_steering_angle_avg(left_lane, right_lane)
        elif (left_lane[0] != 0.0 and left_lane[1] != 0.0) and (right_lane[0] == 0 or right_lane[1] == 0.0):
            cruise_speed = 10.0
            steering_angle = calculate_steering_angle(left_lane)
        elif (left_lane[0] == 0.0 or left_lane[1] == 0.0) and (right_lane[0] != 0 and right_lane[1] != 0.0):
            cruise_speed = 10.0
            steering_angle = calculate_steering_angle_right(right_lane)

        pub_speed.publish(cruise_speed)
        # pub_angle.publish(steering_angle)
        rate.sleep()
        pass
    

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass


