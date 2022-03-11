#!/usr/bin/env python3

""" COMPUTE CONTROL LAWS (CRUISE SPEED & STEERING ANGLE) FOR PASS STATE """

import rospy
from std_msgs.msg import Bool, Float64

# GLOBAL VARIABLES
pass_finished = Bool()
enable_PS = None

# CALLBACK ENABLE PASS
def callback_enable_PS(msg):
    global enable_PS
    enable_PS = msg.data


def main():
    
    global enable_PS, pass_finished

    print('Pass Node')
    rospy.init_node('pass_node')
    rate = rospy.Rate(10)

    # SUBSCRIBERS
    rospy.Subscriber('/enable_PS', Bool, callback_enable_PS)

    # PUBLISHERS
    pub_speed = rospy.Publisher('/goal_cruise_speed', Float64, queue_size=10)
    pub_angle = rospy.Publisher('/goal_steering_angle', Float64, queue_size=10)
    pub_pass_finished = rospy.Publisher('/pass_finished', Bool, queue_size=10)

    while not rospy.is_shutdown():
        pass_finished.data = False
        if enable_PS:
            # COMPUTE STEERING AND SPEED FOR PASSING
            pass_speed = 30.0/10.0                                # COMPUTE SPEED
            steering_angle = -0.1                                 # COMPUTE STEERING ANGLE

            # -----------------------------------------------------

            pub_speed.publish(pass_speed)                         # PUBLISH PASS SPEED
            pub_angle.publish(steering_angle)                     # PUBLISH STEERING ANGLE
            pass_finished.data = True                             # PASS FINISHED
            pub_pass_finished.publish(pass_finished)              # PUBLISH PASS FINISHED
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException
