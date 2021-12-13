#!/usr/bin/env python3

import math
import rospy
import time
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float32

# GLOBAL VARIABLES
current_centroids = []

# PUBLISHERS
pub_vel = rospy.Publisher('/centroid_vel', Float32, queue_size=10)

def callback_object_pose(msg):

    global actual_centroids

    last_centroids = current_centroids.copy()                                                           # GET LAST CENTROIDS
    start_time = time.time()                                                                            # START TIME
    current_centroids.clear()                                                                           # DELETE LAST CENTROIDS
    # print('LAST: ', last_centroids)

    for centroid in msg.poses:
        current_centroids.append([centroid.position.x, centroid.position.y, centroid.position.z])       # GET CURRENT CENTROIDS

    end_time = time.time()                                                                              # END TIME
    final_time = end_time-start_time                                                                    # GET TOTAL TIME BETWEEN LAST-CURRENT
    # print('ACTUAL', actual_centroids)

    # CALCULATE DISTANCE BETWEEN LAST & CURRENT
    if last_centroids:
        print('----')
        for (lc, cc) in zip(last_centroids, current_centroids):
            distance = math.sqrt( (lc[0]-cc[0])**2 + (lc[1]-cc[1])**2 + (lc[2]-cc[2])**2 )
            vel = distance / final_time
            pub_vel.publish(vel)
            print('[last, actual, d, v]; ', [lc, cc, distance, vel] )

def main():

    global velocity, speeds
    print('Object Velocity node ...')
    rospy.init_node('object_velocity')
    rate = rospy.Rate(10)

    # SUBSCRIBERS
    rospy.Subscriber('/centroid_pose', PoseArray, callback_object_pose)
    
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException
