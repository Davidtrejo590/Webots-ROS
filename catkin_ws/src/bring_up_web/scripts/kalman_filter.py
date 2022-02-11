#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseArray


# GLOBAL VARIABLES FOR EKF
#  2D SYSTEM

""" 
        LINEAR DYNAMIC SYSTEM
    x = P_x = P_x + dtV_x + w
        P_y = P_y + dtV_y + w
        V_x = V_x         + w
        V_y = V_y         + w
"""

delta_t = 1/10                                          # SAMPLING TIME
x = np.zeros((4, 1))                                    # INITIAL SYSTEM
w = np.array([                                          # WHITE NOISE
        [0.01],
        [0.01],
        [0.01],
        [0.01]
])
P_k = np.identity(4)                                    # SYSTEM UNCERTAINTY
I_k = np.identity(4)                                    # IDENTITY MATRIX 2x2   
H_k = np.array([                                        # JACOBIAN OF h
    [1, 0, 0, 0],
    [0, 1, 0, 0]
])
R_k = np.array([
    [0.1, 0],
    [0, 0.1]
])                                                      # MOVEMENT NOISE

F_k = np.array([                                        # JACOBIAN OF X SYSTEM
    [1, 0, delta_t, 0],
    [0, 1, 0, delta_t],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

Q_k = np.array([                                        # NOISE FOR SYSTEM, HOW GOOD IT IS?
    [0.01, 0, 0, 0],
    [0, 0.01, 0, 0], 
    [0, 0, 0.01, 0],
    [0, 0, 0, 0.01]
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

    print('X POSITION', [centroid[0], centroid[1]])

    # PREDICT
    x_hat = np.dot(F_k, x) + w                                                  # x' = F * x + U
    P_hat = np.dot(np.dot(F_k, P_k),  np.transpose(F_k)) + Q_k                  # P' = F * P  * F_t + Q

    # print('X_HAT', x_hat)
    # print('P_HAT', P_hat)

    # UPDATE
    z = np.array([
        [centroid[0]],
        [centroid[1]],
    ])
    y = z - np.dot(H_k, x)                                                      # y = z - H * x'
    S_k = np.dot(np.dot(H_k, P_hat), np.transpose(H_k)) + R_k                   # S = H * P' * H_t + R
    K_k = np.dot(np.dot(P_hat, np.transpose(H_k)), np.linalg.inv(S_k))          # K = P' * H_t * S ^-1
    x = x_hat + np.dot(K_k, y)                                                       # x = x' + K * y
    P_k = np.dot(( I_k - np.dot(K_k, H_k)), np.transpose(P_hat))                # P = (I -  k * H) * P'

    print('X PREDICT + UPDATE', [x[0], x[1]])


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
