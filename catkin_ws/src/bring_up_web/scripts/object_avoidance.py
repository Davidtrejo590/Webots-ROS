#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Bool

# STATES
SM_CRUISE   = 'CRUISE'
SM_PASS     = 'PASS'
SM_KEEP     = 'KEEP DISTANCE'

# ENABLES
enable_LT = Bool()                              # ENABLE LANE TRACKING
enable_KD = Bool()                              # ENABLE KEEP DISTANCE
enable_PS = Bool()                              # ENABLE PASSING

# GLOBAL VARIABLES
free_N  = 1
free_W  = 1
free_NW = 1
free_SW = 1
pass_finished = None

# DEFINE BOUNDING BOXES
boundingBoxC = [ round(i, 1) for i in np.arange(  -0.5,  1.50, 0.1 ) ]          # CENTER
boundingBoxW = [ round(i, 1) for i in np.arange(  -4.0,  -2.8, 0.1 ) ]          # WEST
boundingBoxN = [ round(i, 1) for i in np.arange( -20.0,  -9.9, 0.1 ) ]          # NORTH
boundingBoxS = [ round(i, 1) for i in np.arange(  10.0,  20.1, 0.1 ) ]          # SOUTH

# CAR POSE CALLBACK
def callback_car_pose(msg):

    global free_N, free_W, free_NW, free_SW

    cars = []
    for car in msg.poses:
        if car.position.x != 0.0:
            cars.append( [round(car.position.x, 1), round(car.position.z, 1)] )

    # CARS DETECTED
    if cars:
        for c in cars:
            # DEFINE BUSY & FREE REGIONS
            free_N  = free_N   and not ( c[0] in boundingBoxC and c[1] in boundingBoxN ) 
            free_W  = free_W   and not ( c[0] in boundingBoxW and c[1] in boundingBoxC )
            free_NW = free_NW  and not ( c[0] in boundingBoxW and c[1] in boundingBoxN )
            free_SW = free_SW  and not ( c[0] in boundingBoxW and c[1] in boundingBoxS )

        # print('[free_W, free_SW, free_NW, free_N]' , [free_W, free_SW, free_NW, free_N])

# PASS FINISHED CALLBACK
def callback_pass_finished(msg):
    global pass_finished
    pass_finished = msg.data


def main():
    global free_N, free_W, free_NW, free_SW, enable_LT, enable_KD, enable_PS, pass_finished

    # INIT NODE
    print('Object avoidance node...')
    rospy.init_node('object_avoidance')
    rate = rospy.Rate(10)

    # SUBSCRIBERS
    rospy.Subscriber('/filter_pose', PoseArray, callback_car_pose)
    rospy.Subscriber('/pass_finished', Bool, callback_pass_finished)

    # PUBLISHERS
    pub_enable_LT = rospy.Publisher('/enable_LT', Bool, queue_size=10)
    pub_enable_KP = rospy.Publisher('/enable_KD', Bool, queue_size=10)
    pub_enable_PS = rospy.Publisher('/enable_PS', Bool, queue_size=10)

    # STATE MACHINE
    state = SM_CRUISE

    while not rospy.is_shutdown():
        
        # CRUISE STATE
        if state == SM_CRUISE:
            enable_LT.data = True
            enable_KD.data = False
            enable_PS.data = False
            if not free_N and free_W and free_NW:
                state = SM_PASS
            elif not free_N and ( not free_W or not free_NW):
                state = SM_KEEP
            else:
                state ==SM_CRUISE
        # PASS STATE
        elif state == SM_PASS:
            enable_LT.data = False
            enable_KD.data = False
            enable_PS.data = True
            if pass_finished:
                pass_finished = False           # FROM PASS NODE
                state = SM_CRUISE
        # KEEP DISTANCE STATE
        elif state == SM_KEEP:
            enable_LT.data = False
            enable_KD.data = True
            enable_PS.data = False
            if free_N:
                state = SM_CRUISE
            elif free_NW and free_W:
                state = SM_PASS
        

        # PUBLISH ENABLES
        pub_enable_LT.publish(enable_LT)        # ENABLE LANE TRACKING
        pub_enable_KP.publish(enable_KD)        # ENABLE KEEP DISTANCE
        pub_enable_PS.publish(enable_PS)        # ENABLE PASS

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException
