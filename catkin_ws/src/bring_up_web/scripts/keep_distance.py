#!/usr/bin/env python3

""" COMPUTE CONTROL LAWS (CRUISE SPEED & STEERING ANGLE) FOR KEEP DISTANCE STATE """

import rospy
from std_msgs.msg import Bool, Float64, Float64MultiArray
from control_laws import Control

# GLOBAL VARIABLES
left_lane   = [0.0, 0.0]
right_lane  = [0.0, 0.0]
enable_KD = None
distance = None


# LEFT LANE CALLBACK
def callback_left_lane(msg):
    global left_lane
    left_lane = list(msg.data)                                  # TUPLE TO LIST

# RIGHT LANE CALLBACK
def callback_right_lane(msg):
    global right_lane
    right_lane = list(msg.data)                                 # TUPLE TO LIST

# ENABLE KEEP DISTANCE CALLBACK
def callback_enable_KD(msg):
    global enable_KD
    enable_KD = msg.data

# CAR DISTANCE CALLBACK
def callback_distance(msg):
    global distance
    distance = msg.data

# MAIN FUNCTION
def main():

    global enable_KD, distance, left_lane, right_lane

    # CLASS FOR CONTROL LAWS
    control_KD = Control()

    # INIT NODE
    print('Keep Distance Node')
    rospy.init_node('keep_distance')
    rate = rospy.Rate(10)

    # SUBCRIBERS
    rospy.Subscriber('/left_lane', Float64MultiArray, callback_left_lane)
    rospy.Subscriber('/right_lane', Float64MultiArray, callback_right_lane)
    rospy.Subscriber('/enable_KD', Bool, callback_enable_KD)
    rospy.Subscriber('/distance', Float64, callback_distance)
    
    # PUBLISHERS
    pub_speed = rospy.Publisher('/goal_cruise_speed', Float64, queue_size=10)
    pub_angle = rospy.Publisher('/goal_steering_angle', Float64, queue_size=10)


    while not rospy.is_shutdown():
        if enable_KD:                                                   # STATE KEEP DISTANCE
            control_KD.control_law(left_lane, right_lane, 20.0)         # COMPUTE CONTROL LAWS
            pub_speed.publish(control_KD.cruise_speed)                  # PUBLISH CRUISE SPEED
            pub_angle.publish(control_KD.steering_angle)                # PUBLISH STEERING ANGLE

        rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException

