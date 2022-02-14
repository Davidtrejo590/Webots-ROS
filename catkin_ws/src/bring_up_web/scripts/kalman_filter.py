#!/usr/bin/env python3

from turtle import pu
import rospy
import numpy as np
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker, MarkerArray

marker_array = MarkerArray()

# GLOBAL VARIABLES FOR EKF

def callback_object_pose(msg):

    global marker_array
    centroids = []

    for centroid in msg.poses:
        if centroid.position.x != 0.0:
            centroids.append([centroid.position.x, centroid.position.y, centroid.position.z])
    
    # print('MEASUERENTS')
    # for c in centroids:
    #     print([c[0], c[1]])

    estimate = [ [] for i in centroids ]
    for i in range(len(centroids)):    
        estimate[i] = ekf(centroids[i])
    
    if estimate:
        i = 0
        print('ESTIMSTED')
        for e in estimate:
            marker = Marker()
            marker.header.frame_id = 'lidar_link'
            marker.ns = 'marker' + str(i)
            marker.id = i
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.text = 'p[x, z] = ' + str(np.round(e[0])) + str(np.round(e[1], 2)) + '\n' 'v[x, z] = ' + str(np.round(e[2])) + str(np.round(e[3], 2))
            marker.pose.position.x = e[0]
            marker.pose.position.y = e[1]
            marker.pose.position.z = centroids[i][2]
            marker.scale.z = 0.3
            marker.color.r, marker.color.g, marker.color.b = [1.0, 1.0, 1.0]
            marker.color.a = 1.0
            marker_array.markers.append(marker)
            i +=1
    else:
        print('NOT ESTIMATED')


    # print('ESTIMATES')
    # for e in estimate:
    #     print(e[0], e[1])
        

    
    # centroid = [ msg.poses[0].position.x, msg.poses[0].position.y, msg.poses[0].position.z ]
    # [x, y] = ekf(centroid)
    
    



# KALMAN FILTER
def ekf( centroid ):

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


    # print('X POSITION', [centroid[0], centroid[1]])

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

    # print('X PREDICT + UPDATE', [x[0], x[1]]) 

    return x

# KALMAN FILTER 2D



def main():

    global marker_array

    print('Kalman Filter Node...')
    rospy.init_node('object_velocity')
    rate = rospy.Rate(10)

    # SUBSCRIBERS
    rospy.Subscriber('/object_pose', PoseArray, callback_object_pose)

    # PUBLISHER
    pub_marker = rospy.Publisher('/marker', MarkerArray, queue_size=10)
    

    while not rospy.is_shutdown():
        pub_marker.publish(marker_array)
        marker_array.markers.clear()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException






