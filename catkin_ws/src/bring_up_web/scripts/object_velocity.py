#!/usr/bin/env python3

import math
import rospy
import time
import numpy as np
from geometry_msgs.msg import PoseArray


# GLOBAL VARIABLES FOR EKF
#  X COMPONENT

""" 
        LINEAR DYNAMIC SYSTEM
    x = P_x = P_x + dtV_y + w
        V_x = V_x         + w
"""

delta_t = 1/10                                          # SAMPLING TIME
x = np.zeros((2, 1))                                    # INITIAL SYSTEM
w = np.array([                                          # WHITE NOISE
        [0.01],
        [0.01]
])
P_k = np.identity(2)                                    # SYSTEM UNCERTAINTY
I_k = np.identity(2)                                    # IDENTITY MATRIX 2x2   
H_k = np.array([                                        # JACOBIAN OF h
    [1, 0]
])
R_k = 0.1                                               # MOVEMENT NOISE

F_k = np.array([                                        # JACOBIAN OF X SYSTEM
    [1, delta_t],
    [0, 1]
])

Q_k = np.array([                                        # NOISE FOR SYSTEM, HOW GOOD IT IS?
    [0.01, 0],
    [0, 0.01]
])


def callback_object_pose(msg):

    # for centroid in msg.poses:
    #     if centroid.position.x != 0.0:
    #         print([centroid.position.x, centroid.position.y, centroid.position.z])
    #         ekf(centroid.position.x)
    centroid = [ msg.poses[0].position.x, msg.poses[0].position.y, msg.poses[0].position.z ]
    ekf(centroid)



# KALMAN FILTER
def ekf( centroid ):
    global x, P_k

    print('X POSITION', centroid[0])

    # PREDICT
    x_hat = np.dot(F_k, x) + w                                                  # x' = F * x + U
    P_hat = np.dot(np.dot(F_k, P_k),  np.transpose(F_k)) + Q_k                  # P' = F * P  * F_t + Q

    # print('X_HAT', x_hat)
    # print('P_HAT', P_hat)

    # UPDATE
    y = centroid[0] - x[0]                                                      # y = z - H * x'
    S_k = np.dot(np.dot(H_k, P_hat), np.transpose(H_k)) + R_k                   # S = H * P' * H_t + R
    K_k = np.dot(P_hat, np.transpose(H_k)) * (1/S_k)                            # K = P' * H_t * S ^-1
    x = x_hat + (K_k * y)                                                       # x = x' + K * y
    P_k = np.dot(( I_k - np.dot(K_k, H_k)), np.transpose(P_hat))                # P = (I -  k * H) * P'

    print('X PREDICT + UPDATE', x[0])


# KALMAN FILTER 2D



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
