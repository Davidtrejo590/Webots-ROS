#!/usr/bin/env python3

"""
    NODE TO CHOOSE THE CORRECT BEHAVIOR (REFEREE SYSTEM)
    (PASS, CRUISE, KEEP DISTANCE), ENABLE THE CORRESPOND STATE
"""

# LIBRARIES
import rospy
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Bool, Float64

# STATES
SM_LANE_TRACKING    = 'SM_LANE_CRUISE'
SM_WAIT_PASS        = 'SM_WAIT_PASS'
SM_FINISH           = 'SM_FINISH'
SM_CRUISE           = 'CRUISE'
SM_PASS             = 'PASS'
SM_KEEP             = 'KEEP DISTANCE'
SM_INIT             = 'SM_INIT'

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

    global free_N, free_W, free_NW, free_SW, pass_finished, safe_distance

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
    state = SM_INIT

    while not rospy.is_shutdown():

        if state == SM_INIT:                                # STATE INIT 
            state = SM_CRUISE

        elif state == SM_CRUISE:                            # STATE LANE TRACKING
            if not free_N and free_W and free_NW:
                state = SM_PASS
            elif not free_N and ( not free_W or not free_NW):
                state = SM_KEEP
            else:
                pub_enable_LT.publish(True)                 # ENABLE LANE TRACKING
                state = SM_CRUISE

        elif state == SM_PASS:                              # STATE OVERTAKE
            pub_enable_PS.publish(True)                     # START OVERTAKE
            pub_enable_LT.publish(False)                    # DISABLE LANE TRACKING
            pub_enable_KD.publish(False)                    # DISABLE LANE TRACKING
            state = SM_WAIT_PASS

        elif state == SM_WAIT_PASS:                         # STATE WAIT OVERTAKE
            pub_enable_PS.publish(False)                    # START OVERTAKE
            free_N  = 1
            free_W  = 1
            free_NW = 1
            free_SW = 1
            if pass_finished:
                state = SM_FINISH
            else:
                state = SM_WAIT_PASS

        elif state == SM_KEEP:                              # KEEP DISTANCE STATE
            pub_front_car.publish(safe_distance)            # PUBLISH FRONT CAR
            pub_enable_PS.publish(False)                    # START OVERTAKE
            pub_enable_LT.publish(False)                    # DISABLE LANE TRACKING
            pub_enable_KD.publish(True)                     # DISABLE LANE TRACKING
            if free_N:
                state = SM_CRUISE
            elif free_NW and free_W:
                state = SM_PASS

        elif state == SM_FINISH:                            # STATE FINISH
            pass_finished = False
            state = SM_CRUISE
            
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException
        pass
