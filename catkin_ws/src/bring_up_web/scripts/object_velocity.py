#!/usr/bin/env python3

import math
import rospy
import time
from geometry_msgs.msg import PoseArray

# GLOBAL VARIABLES
current_centroids = []

def callback_object_pose(msg):

    last_centroids = current_centroids.copy()                                                           # GET LAST CENTROIDS
    start_time = time.time()                                                                            # START TIME
    current_centroids.clear()                                                                           # DELETE LAST CENTROIDS
    # print('LAST: ', last_centroids)

    for centroid in msg.poses:
        current_centroids.append([centroid.position.x, centroid.position.y, centroid.position.z])       # GET CURRENT CENTROIDS

    end_time = time.time()                                                                              # END TIME
    final_time = end_time-start_time                                                                    # GET TOTAL TIME BETWEEN LAST-CURRENT
    # print('ACTUAL', current_centroids)

    # CALCULATE DISTANCE BETWEEN LAST & CURRENT
    if last_centroids:
        print('----')
        for (lc, cc) in zip(last_centroids, current_centroids):
            if lc[0] != 0.0:
                distance = math.sqrt( (lc[0]-cc[0])**2 + (lc[1]-cc[1])**2 + (lc[2]-cc[2])**2 )
                vel = distance / final_time
                # print('[last, actual, d, v]; ', [lc, cc, distance, vel] )


# KALMAN FILTER
def predict():
    print('PREDICT')
    # g -> g (x_t, u)
    # G -> j(g)
    # sigma = G * sigma * Gt + Q


def update():
    print('UPDATE')
    # z -> MEDICIONES REALES
    # z_hat = h(x, S)   -> LECTURAS PREVISTAS
    # H = J(h)

    # k = sigma * Ht * (1/ (H * sigma * Ht + R))            -> KALMAN GAIN

    # ACTUALIZAR VECTOR DE ESTADO Y MATRIZ DE COVARIANZA
    # x = x + k( z - z_hat)
    # sigma = (I - (k * H )) * sigma

def main():
    print('Kalman Filter Node...')
    rospy.init_node('object_velocity')
    rate = rospy.Rate(10)

    # SUBSCRIBERS
    rospy.Subscriber('/object_pose', PoseArray, callback_object_pose)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException
