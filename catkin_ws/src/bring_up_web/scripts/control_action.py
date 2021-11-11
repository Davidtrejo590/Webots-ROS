#!/usr/bin/env python3

""" CALCULATE CONTROL ACTIONS (CRUISE SPEED & STEERING ANGLE) """

# LIBRARIES
from numpy.lib.function_base import angle
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray


# GLOBAL VARIABLES
left_lane = [0,0]
right_lane = [0,0]

# LEFT LANE CALLBACK
def callback_left_lane(msg):
    global left_lane
    left_lane = msg.data

# RIGHT LANE CALLBACK
def callback_right_lane(msg):
    global right_lane
    right_lane = msg.data

def calculate_steering_angle(lane):
    goal_distance = 146.24038429927623
    goal_angle = 0.646238328234549
    kd = 0.01
    ka = 0.01
    steering = 0.0
    detected_distace, detected_angle = lane
    # print('Distance Detected: ', detected_distace)
    # print('Angle Detected: ', detected_angle)

    # if detected_angle < goal_angle:
    #     steering = -0.0174533
    # elif detected_angle > goal_angle:
    #     steering = 0.0174533
    
    ed = goal_distance - detected_distace
    ea = goal_angle - detected_angle
    steering = (kd * ed) + (ka * ea)
    print('Steering Calculated: ', steering)
    return steering


# MAIN FUNCTION
def main():
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
        if( (left_lane[0] == 0 and left_lane[1] == 0) or (right_lane[0] == 0 and right_lane[1] == 0) ):
            cruise_speed = 0.0
        else:                                                                                           # THERE ARE LINES
            cruise_speed = 10.0                                                                         # SET CRUISE SPEED TO 10 km/h
            steering_angle = calculate_steering_angle(left_lane)
        pub_speed.publish(cruise_speed)
        pub_angle.publish(steering_angle)
        rate.sleep()
        pass
    

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

