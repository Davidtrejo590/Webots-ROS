#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Bool, Float64

# STATES
SM_CRUISE   = 'CRUISE'
SM_PASS     = 'PASS'
SM_KEEP     = 'KEEP DISTANCE'

# ENABLES
enable_LT = Bool()              # ENABLE LANE TRACKING
enable_KD = Bool()              # ENABLE KEEP DISTANCE
enable_PS = Bool()              # ENABLE PASSING

# GLOBAL VARIABLES
free_N  = 1
free_W  = 1
free_NW = 1
free_SW = 1
pass_finished = False
safe_distance = 0.0

# DEFINE BOUNDING BOXES
boundingBoxC = [ round(i, 1) for i in np.linspace(  -0.5,   1.50 ) ]          # CENTER
boundingBoxW = [ round(i, 1) for i in np.linspace(  -4.0,   -2.5 ) ]          # WEST
boundingBoxN = [ round(i, 1) for i in np.linspace( -20.0,  -10.0 ) ]          # NORTH
boundingBoxS = [ round(i, 1) for i in np.linspace(  10.0,   20.0 ) ]          # SOUTH

# CAR POSE CALLBACK
def callback_car_pose(msg):

    global free_N, free_W, free_NW, free_SW, safe_distance

    cars = []
    for car in msg.poses:
        # print([ round(car.position.x, 1), round(car.position.z, 1) ])
        cars.append( [ round(car.position.x, 1), round(car.position.z, 1) ] )

    # CARS DETECTED
    if cars:
        for c in cars:
            # DEFINE BUSY & FREE REGIONS
            # free_N  = free_N   and not ( ( c[0] > -0.5 and c[0] <  1.5 ) and ( c[1] > -20.0 and c[1] < -10.0 ) ) 
            # free_NW = free_NW  and not ( ( c[0] > -4.0 and c[0] < -2.5 ) and ( c[1] > -20.0 and c[1] < -10.0 ) )
            # free_W  = free_W   and not ( ( c[0] > -4.0 and c[0] < -2.5 ) and ( c[1] >  -0.5 and c[1] <   1.5 ) )
            # free_SW = free_SW  and not ( ( c[0] > -4.0 and c[0] < -2.5 ) and ( c[1] >  10.0 and c[1] <  20.0 ) )

            free_N  = free_N   and not ( c[0] in boundingBoxC and c[1] in boundingBoxN ) 
            free_W  = free_W   and not ( c[0] in boundingBoxW and c[1] in boundingBoxC )
            free_NW = free_NW  and not ( c[0] in boundingBoxW and c[1] in boundingBoxN )
            free_SW = free_SW  and not ( c[0] in boundingBoxW and c[1] in boundingBoxS )

            # GET CAR IN FRONT
            # if not free_N:
            #     # CAR POSITION [Z]
            #     safe_distance = c[1]
            #     print('CAR IN FRONT', c)
                
            # else:
            #     safe_distance = 0.0

        # print('[free_W, free_SW, free_NW, free_N]' , [free_W, free_SW, free_NW, free_N])


# PASS FINISHED CALLBACK
def callback_pass_finished(msg):
    global pass_finished
    pass_finished = msg.data


def main():
    global free_N, free_W, free_NW, free_SW, enable_LT, enable_KD, enable_PS, pass_finished, safe_distance

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
    pub_front_car = rospy.Publisher('/safe_distance', Float64, queue_size=10)

    # STATE MACHINE
    state = SM_CRUISE

    while not rospy.is_shutdown():
        
        # CRUISE STATE
        if state == SM_CRUISE:
            print('CRUISE STATE')
            enable_LT.data = True
            enable_KD.data = False
            enable_PS.data = False
            if not free_N and free_W and free_NW:
                state = SM_PASS
            elif not free_N and ( not free_W or not free_NW):
                state = SM_KEEP
            else:
                state = SM_CRUISE
        # PASS STATE
        elif state == SM_PASS:
            print('PASS STATE')
            enable_LT.data = False
            enable_KD.data = False
            enable_PS.data = True
            free_N = 1
            free_W = 1
            free_NW = 1
            free_SW = 1
            if pass_finished:
                pass_finished = False           # FROM PASS NODE
                state = SM_CRUISE
            else:
                state = SM_PASS
        # KEEP DISTANCE STATE
        elif state == SM_KEEP:
            print('KEEP DISTANCE STATE')
            enable_LT.data = False
            enable_KD.data = True
            enable_PS.data = False
            pub_front_car.publish(safe_distance)    # PUBLISH FRONT CAR
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
