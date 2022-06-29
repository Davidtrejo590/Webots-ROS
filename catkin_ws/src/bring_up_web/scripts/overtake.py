#!/usr/bin/env python3

""" 
    ENABLE AND COMPUTE CONTROL LAWS (CRUISE SPEED & STEERING ANGLE) FOR OVERTAKE STATE 
"""

from ast import Pass
import rospy
from std_msgs.msg import Bool, Float64

# STATES
SM_WAIT_NEW_OVERTAKE    = 'SM_WAIT_NEW_OVERTAKE'
SM_WAIT_TURN_LEFT       = 'SM_WAIT_TURN_LEFT'
SM_WAIT_TURN_RIGHT      = 'SM_WAIT_TURN_RIGHT'
SM_WAIT_GO_STRAIGHT     = 'SM_WAIT_STRAIGHT'
SM_WAIT_ALIGN_RIGHT     = 'SM_WAIT_ALIGN_RIGHT'
SM_WAIT_ALIGN_LEFT      = 'SM_WAIT_ALIGN_LEFT'
SM_FINISH_OVERTAKE      = 'SM_FINISH_OVERTAKE'
SM_TURN_LEFT            = 'SM_TURN_LEFT'
SM_TURN_RIGHT           = 'SM_TURN_RIGHT'
SM_GO_STRAIGHT          = 'SM_GO_STRAIGHT'
SM_ALIGN_RIGHT          = 'SM_ALIGN_RIGHT'
SM_ALIGN_LEFT           = 'SM_ALIGN_LEFT'
SM_START                = 'SM_START'

# GLOBAL VARIABLES
enable_PS           = None
current_steering    = None
dynamic             = None 

# CALLBACK ENABLE PASS
def callback_enable_PS(msg):
    global enable_PS
    enable_PS = msg.data

def callback_current_steering(msg):
    global current_steering
    current_steering = msg.data

# MAIN FUNCTION
def main():
    
    global enable_PS, current_steering, dynamic
    
    # INIT NODE
    print('Pass Node...')
    rospy.init_node('pass_node')
    rate = rospy.Rate(10)

    # PARAMS 
    if rospy.has_param('/dynamic'):
        dynamic = rospy.get_param('/dynamic')

    # SUBSCRIBERS
    rospy.Subscriber('/enable_PS', Bool, callback_enable_PS)
    rospy.Subscriber('/current_steering', Float64, callback_current_steering)

    # PUBLISHERS
    pub_enable_right_light = rospy.Publisher('/enable_right_light', Bool, queue_size=10)
    pub_enable_left_light  = rospy.Publisher('/enable_left_light', Bool, queue_size=10)
    pub_pass_finished   = rospy.Publisher('/pass_finished', Bool, queue_size=10)
    pub_steering        = rospy.Publisher('/goal_steering', Float64, queue_size=10)
    pub_speed           = rospy.Publisher('/goal_speed', Float64, queue_size=10)
    
    
    # STATE MACHINE TO OVERTAKE
    state   = SM_START
    i       = 0
    count   = 0

    while not rospy.is_shutdown():
        if state == SM_START:                                   # STATE START
            state = SM_WAIT_NEW_OVERTAKE
        
        elif state == SM_WAIT_NEW_OVERTAKE:                     # STATE WAIT NEW OVERTAKE
            if enable_PS:
                i += 1
                pub_steering.publish(0.0)
                pub_speed.publish(20.0)
                state = SM_TURN_LEFT
            else:
                state = SM_WAIT_NEW_OVERTAKE
        
        elif state == SM_TURN_LEFT:                             # STATE TURN LEFT
            pub_steering.publish(current_steering - 0.35)
            count = 0
            state = SM_WAIT_TURN_LEFT

        elif state == SM_WAIT_TURN_LEFT:                        # STATE WAIT TURN LEFT
            count += 1
            pub_enable_left_light.publish(True)
            pub_enable_right_light.publish(False)
            if count > 10:
                state = SM_ALIGN_RIGHT
            else:
                state = SM_WAIT_TURN_LEFT

        elif state == SM_ALIGN_RIGHT:                           # STATE TURN RIGHT
            pub_steering.publish(current_steering + 0.35)
            count = 0
            state = SM_WAIT_ALIGN_RIGHT

        elif state == SM_WAIT_ALIGN_RIGHT:                      # STATE WAIT TURN RIGHT
            count += 1
            pub_enable_left_light.publish(False)
            pub_enable_right_light.publish(False)

            if i == 1 and dynamic:
                if count > 0:
                    pub_steering.publish(0.0)
                    pub_speed.publish(30.0)
                    x = 0
                    while x < 65:
                        x += 1
                    state = SM_TURN_RIGHT
                else: 
                    state = SM_WAIT_ALIGN_RIGHT
            else:
                if count > 0:
                    state = SM_TURN_RIGHT
                else:
                    state = SM_WAIT_ALIGN_RIGHT

        elif state == SM_TURN_RIGHT:                            # STATE TURN RIGHT 2
            pub_steering.publish(current_steering + 0.6)
            count = 0
            state = SM_WAIT_TURN_RIGHT
        
        elif state == SM_WAIT_TURN_RIGHT:                       # STATE WAIT TURN RIGHT 2
            count += 1
            pub_enable_left_light.publish(False)
            pub_enable_right_light.publish(True)

            if count > 8:
                state = SM_ALIGN_LEFT
            else:
                state = SM_WAIT_TURN_RIGHT
        
        elif state == SM_ALIGN_LEFT:                            # STATE TURN LEFT 2
            pub_steering.publish(current_steering - 0.6)
            count = 0
            state = SM_WAIT_ALIGN_LEFT

        elif state == SM_WAIT_ALIGN_LEFT:                       # STATE WAIT TURN LEFT 2
            count += 1
            pub_enable_left_light.publish(False)
            pub_enable_right_light.publish(False)

            if count > 5:
                state = SM_GO_STRAIGHT
            else:
                state = SM_WAIT_ALIGN_LEFT

        elif state == SM_GO_STRAIGHT:
            pub_steering.publish(0.0)
            pub_speed.publish(0.0)
            count = 0
            state = SM_WAIT_GO_STRAIGHT

        elif state == SM_WAIT_GO_STRAIGHT:
            count += 1
            if count > 0:
                state = SM_FINISH_OVERTAKE
            else:
                state = SM_WAIT_GO_STRAIGHT

        elif state == SM_FINISH_OVERTAKE:                       # STATE FINISH OVERTAKE
            pub_pass_finished.publish(True)
            state = SM_WAIT_NEW_OVERTAKE        
            
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException
        pass
