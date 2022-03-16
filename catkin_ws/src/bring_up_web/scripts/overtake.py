#!/usr/bin/env python3

""" COMPUTE CONTROL LAWS (CRUISE SPEED & STEERING ANGLE) FOR PASS STATE """

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
        if enable_PS:

            # PASSING ACTION

            # pub_speed.publish(pass_speed)                         # DISMINUIR VELOCIDAD

            steering_angle = -( 0.0174533 * 6 )                     # GIRAR A LA IZQUIERDA
            pub_angle.publish(steering_angle)
            time.sleep(2)

            steering_angle = ( 0.0174533 * 6 )                      # GIRAR A LA DERECHA
            pub_angle.publish(steering_angle)
            time.sleep(1)

            steering_angle = 0.0                                    # MANTENERSE
            pub_angle.publish(steering_angle)
            time.sleep(2)

            steering_angle = ( 0.0174533 * 12 )                     # GIRAR A LA DERECHA
            pub_angle.publish(steering_angle)
            time.sleep(2)

            steering_angle = -( 0.0174533 * 12 )                    # GIRAR A LA IZQUIERDA PARA ALINEAR
            pub_angle.publish(steering_angle)
            time.sleep(1)

            pass_finished.data = True                               # REBASE TERMINADO 
            print('PASS FINISHED')

            pub_pass_finished.publish(pass_finished)                # PUBLISH PASS FINISHED
            
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException
