#!/usr/bin/env python3

# LIBRARIES
import keyboard
import rospy
from std_msgs.msg import Float64
import keyboard as key

# CONSTANTS
TIME_STEP = 64

# GLOBAL VARIABLES
current_angle = 0.0

# CURRENT ANGLE CALLBACK
def callback_current_angle( msg ):
    global current_angle
    current_angle = msg.data
    print('Current car angle: ', msg.data) 

def check_keyboard():
    print('Enter a key: ')
    if key.read_key() == "a":
        print(' a has been pressed')
    elif key.read_key() == "d":
        print(' d has been pressed')
    elif key.read_key() == "w":
        print(' w has been pressed')
    elif key.read_key() == "s":
        print(' s has been pressed')
    else:
        pass

def main():
    # INIT NODE
    print('INITIALIZING ANGLE NODE ...')
    rospy.init_node('angle_node')
    # SUBSCRIPTIONS
    rospy.Subscriber('/pub_current_angle', Float64, callback_current_angle)
    rate = rospy.Rate(10)

    # CREATE PUBLISHER
    pub_angle = rospy.Publisher('pub_angle', Float64, queue_size=10)

    while not rospy.is_shutdown():
        check_keyboard()
        rate.sleep()
        pass


if __name__ == "__main__":
    main()