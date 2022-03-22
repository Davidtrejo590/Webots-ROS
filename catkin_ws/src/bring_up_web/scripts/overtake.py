#!/usr/bin/env python3

""" 
    ENABLE AND COMPUTE CONTROL LAWS (CRUISE SPEED & STEERING ANGLE) FOR PASS STATE 
"""

import rospy
from std_msgs.msg import Bool, Float64
import time

# GLOBAL VARIABLES
pass_finished = Bool()
enable_PS = None

# CALLBACK ENABLE PASS
def callback_enable_PS(msg):
    global enable_PS
    enable_PS = msg.data

# MAIN FUNCTION
def main():
    
    global enable_PS, pass_finished
    
    print('Pass Node...')
    rospy.init_node('pass_node')
    rate = rospy.Rate(10)

    # SUBSCRIBERS
    rospy.Subscriber('/enable_PS', Bool, callback_enable_PS)

    # PUBLISHERS
    pub_angle = rospy.Publisher('/goal_steering_angle', Float64, queue_size=10)
    pub_pass_finished = rospy.Publisher('/pass_finished', Bool, queue_size=10)

    while not rospy.is_shutdown():
        # PASSING ACTION
        if enable_PS:

            steering_angle = -( 0.0174533 * 6 )            # TURN LEFT
            pub_angle.publish(steering_angle)
            time.sleep(2)

            steering_angle = ( 0.0174533 * 6 )             # TURN RIGHT
            pub_angle.publish(steering_angle)
            time.sleep(1)

            steering_angle = 0.0                           # STAY STRAIGHT
            pub_angle.publish(steering_angle)
            time.sleep(2.5)

            steering_angle = ( 0.0174533 * 12 )            # TURN RIGHT
            pub_angle.publish(steering_angle)
            time.sleep(2)

            steering_angle = -( 0.0174533 * 12 )           # TURN LEFT
            pub_angle.publish(steering_angle)
            time.sleep(1)

            pass_finished.data = True                      # PASS FINISHED

            pub_pass_finished.publish(pass_finished)       # PUBLISH PASS FINISHED
            
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException
