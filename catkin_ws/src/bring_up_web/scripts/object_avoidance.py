#!/usr/bin/env python3

"""
    NODE TO CREATE A STATES MACHINE AND CHOOSE THE CORRECT BEHAVIOR
    (PASS, CRUISE, KEEP DISTANCE), ENABLE THE CORRESPOND STATE
"""

# LIBRARIES
import rospy
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

# CAR POSE CALLBACK
def callback_car_pose(msg):

    global free_N, free_W, free_NW, free_SW, safe_distance

    cars = []
    for car in msg.poses:
        cars.append( [ round(car.position.x, 1), round(car.position.z, 1) ] )

    # CARS DETECTED
    if cars:
        for c in cars:
            # DEFINE BUSY & FREE REGIONS (BOUNDING BOXES)
            free_N  = free_N   and not ( ( c[0] > -0.5 and c[0] <  1.5 ) and ( c[1] > -15.0 and c[1] <  0.0 ) ) # CN 
            free_NW = free_NW  and not ( ( c[0] > -4.0 and c[0] < -2.5 ) and ( c[1] > -15.0 and c[1] <  0.0 ) ) # NW
            free_W  = free_W   and not ( ( c[0] > -4.0 and c[0] < -2.5 ) and ( c[1] >  -0.5 and c[1] <  1.5 ) ) # CW
            free_SW = free_SW  and not ( ( c[0] > -4.0 and c[0] < -2.5 ) and ( c[1] >  10.0 and c[1] < 15.0 ) ) # SW

            # GET CAR IN FRONT
            if not free_N and ( not free_W or not free_NW):
                safe_distance = c[1]
            else:
                safe_distance = 0.0



# PASS FINISHED CALLBACK
def callback_pass_finished(msg):
    global pass_finished
    pass_finished = msg.data

# MAIN FUNCTION
def main():
    global free_N, free_W, free_NW, free_SW, enable_LT, enable_KD, enable_PS, pass_finished, safe_distance

    # INIT NODE
    print('Object Avoidance Node...')
    rospy.init_node('object_avoidance')
    rate = rospy.Rate(10)

    # SUBSCRIBERS
    rospy.Subscriber('/filter_pose', PoseArray, callback_car_pose)
    rospy.Subscriber('/pass_finished', Bool, callback_pass_finished)

    # PUBLISHERS
    pub_enable_LT = rospy.Publisher('/enable_LT', Bool, queue_size=10)
    pub_enable_KD = rospy.Publisher('/enable_KD', Bool, queue_size=10)
    pub_enable_PS = rospy.Publisher('/enable_PS', Bool, queue_size=10)
    pub_front_car = rospy.Publisher('/safe_distance', Float64, queue_size=10)

    # STATE MACHINE
    state = SM_CRUISE

    while not rospy.is_shutdown():
        
        if state == SM_CRUISE:                      # CRUISE STATE
            print('CRUISE')
            enable_LT.data = True
            enable_KD.data = False
            enable_PS.data = False
            if not free_N and free_W and free_NW:
                state = SM_PASS
            elif not free_N and ( not free_W or not free_NW):
                state = SM_KEEP
        
        elif state == SM_PASS:                      # PASS STATE
            print('PASS')
            enable_PS.data = True
            enable_LT.data = False
            enable_KD.data = False
            free_N = 1
            free_W = 1
            free_NW = 1
            free_SW = 1
            if pass_finished:
                pass_finished = False               # FROM PASS NODE
                state = SM_CRUISE

        elif state == SM_KEEP:                      # KEEP DISTANCE STATE
            print('KEEP DISTANCE')
            pub_front_car.publish(safe_distance)    # PUBLISH FRONT CAR
            enable_KD.data = True
            enable_LT.data = False
            enable_PS.data = False
            if free_N:
                state = SM_CRUISE
            elif free_NW and free_W:
                state = SM_PASS
        

        # PUBLISH ENABLES
        pub_enable_LT.publish(enable_LT)        # ENABLE LANE TRACKING
        pub_enable_KD.publish(enable_KD)        # ENABLE KEEP DISTANCE
        pub_enable_PS.publish(enable_PS)        # ENABLE PASS

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException
