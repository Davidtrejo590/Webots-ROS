#!/usr/bin/env python3

""" CALCULATE CONTROL ACTIONS (CRUISE & SPEED) """

# LIBRARIES
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray


# GLOBAL
left_lane = []
right_lane = []


def callback_left_lane(msg):
    global left_lane
    left_lane = msg.data


def callback_right_lane(msg):
    global right_lane
    right_lane = msg.data


def main():
    cruise_speed = 0.0

    # INIT NODE
    print('INITIALIZING CONTROL ACTION ...')
    rospy.init_node('control_action')
    rate = rospy.Rate(10)  # 10Hz

    # SUBSCRIBERS
    rospy.Subscriber('/left_lane', Float64MultiArray, callback_left_lane)
    rospy.Subscriber('/right_lane', Float64MultiArray, callback_right_lane)

    # PUBLISHERS
    pub_speed = rospy.Publisher('/goal_cruise_speed', Float64, queue_size=10)


    while not rospy.is_shutdown():
        if( len(left_lane) != 0 and len(right_lane) !=0 ):
            cruise_speed = 10.0
        else:
            cruise_speed = 0.0
        pub_speed.publish(cruise_speed)
        rate.sleep()
        pass
    
    # rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

