#!/usr/bin/env python3

# LIBRARIES
import rospy
from std_msgs.msg import Float64

# CONSTANTS
MAX_SPEED = 20.0
cruise_speed = 0.0

def main():
    global cruise_speed
    # INIT NODE
    print('INITIALIZING SPEED NODE ...')
    rospy.init_node('speed_node')
    rate = rospy.Rate(10)  # 10Hz
    # CREATE PUBLISHER
    pub_speed = rospy.Publisher('/pub_speed', Float64, queue_size=10)

    # MAIN LOOP
    while not rospy.is_shutdown():
        if( cruise_speed >= MAX_SPEED ):
            cruise_speed = MAX_SPEED
        else:
            cruise_speed += 0.5
        # rospy.loginfo(cruise_speed)
        pub_speed.publish(cruise_speed)                                     # PUBLISHING SPEED
        rate.sleep()
        pass


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

