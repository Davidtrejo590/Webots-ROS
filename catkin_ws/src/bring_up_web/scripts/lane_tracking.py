#!/usr/bin/env python3

""" 
    NODE TO ENABLE CONTROL LAWS (CRUISE SPEED & STEERING ANGLE) FOR LANE TRACKING BEHAVIOR
"""

# LIBRARIES
import rospy
from std_msgs.msg import Float64, Bool, Float64MultiArray
from control_laws import Control


# GLOBAL VARIABLES
left_lane   = [0.0, 0.0]
right_lane  = [0.0, 0.0]
enable_LT   = None

# LEFT LANE CALLBACK
def callback_left_lane(msg):
    global left_lane
    left_lane = list(msg.data)          # TUPLE TO LIST

# RIGHT LANE CALLBACK
def callback_right_lane(msg):
    global right_lane
    right_lane = list(msg.data)        # TUPLE TO LIST

# ENABLE LANE TRACKIG CALLBACK
def callback_enable_LT(msg):
    global enable_LT
    enable_LT = msg.data

# MAIN FUNCTION
def main():

    global left_lane, right_lane, enable_LT

    # CLASS FOR CONTROL LAWS
    control_LT = Control()

    # INIT NODE
    print('Lane Tracking Node...')
    rospy.init_node('lane_tracking')
    rate = rospy.Rate(10)

    # SUBSCRIBERS
    rospy.Subscriber('/left_lane', Float64MultiArray, callback_left_lane)
    rospy.Subscriber('/right_lane', Float64MultiArray, callback_right_lane)
    rospy.Subscriber('/enable_LT', Bool, callback_enable_LT)

    # PUBLISHERS
    pub_speed = rospy.Publisher('/goal_cruise_speed', Float64, queue_size=10)
    pub_angle = rospy.Publisher('/goal_steering_angle', Float64, queue_size=10)


    while not rospy.is_shutdown():
        if enable_LT:                                               # LANE TRACKING STATE
            control_LT.control_law(left_lane, right_lane)           # COMPUTE CONTROL LAWS
            pub_speed.publish(control_LT.cruise_speed)              # PUBLISH CRUISE SPEED
            pub_angle.publish(control_LT.steering_angle)            # PUBLISH STEERING ANGLE
        
        rate.sleep()
    

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass


