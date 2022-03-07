#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseArray

# CONSTANTS 
MIN_DISTANCE = 3.0
MAX_DISTANCE = 5.0

def callback_centroid_pose(msg):

    obstacles = []

    for obstacle in msg.poses:
        if obstacle.position.x != 0.0:
            # OBSTACLE[XP, ZP, XV, ZV]
            obstacles.append([obstacle.position.x, obstacle.position.z, obstacle.orientation.x, obstacle.orientation.z]) 
    
    # BEHAVIORS STATES MACHINE
    choose_behavior(obstacles)


def choose_behavior( obstacles ):
    print('Behaviors')
    
    if obstacles:                                                     # THERE ARE OBSTACLES
        for obstacle in obstacles:
            if obstacle[1] > MIN_DISTANCE:
                print('STATE - 0 / CRUISE BEHAVIOR')   
            elif obstacle[1] < MIN_DISTANCE:                          # Z - DISTANCE / CAR IN FRONT
                print('STATE - 1 / CAR IN FRONT')
                for obs in obstacles:
                    if obs[0] < 3.0 and obs[1] < MIN_DISTANCE:
                        print('CAR IN FRONT AND LEFT - STATE 2')
                        print('BEHAVIOR FOLLOW CAR')
                    elif obs[0] < -3.0 and obs[1] < 0.5:
                        print('CAR ON THE LEFT - STATE 3')
                        print('BEHAVIOR FOLLOW CAR')
                    elif obs[0] < -3.0 and obs[1] > MAX_DISTANCE:
                        print('CAR LEFT AND AWAY - STATE 4')
                        print('BEHAVIOR PASSING ACTION')
    else:                                                             # THERE AREN'T OBSTACLES
        print('STATE - 0 / CRUISE BEHAVIOR')

                        


def main():

    # INIT NODE
    print('Object avoidance node...')
    rospy.init_node('object_avoidance')
    rate = rospy.Rate(10)

    # SUBSCRIBERS
    rospy.Subscriber('/filter_pose', PoseArray, callback_centroid_pose)
    rospy.spin()



if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException




