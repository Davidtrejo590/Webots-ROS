#!/usr/bin/env python3

import math
import rospy
import time
import numpy as np
from geometry_msgs.msg import PoseArray
from kalman_filter import Kalman


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
    print('POSITION', [centroid[0], centroid[1]])
    # # ekf(centroid)
    # k = Kalman(centroid[0], centroid[1])
    # [x, y]= k.ekf()
    # print([x, y])



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


# RESPALDO


#  global marker_array, pose_array, centroids, filters, flag
#     last_centroids = centroids.copy()                                                          # LAST CENTROIDS
#     centroids.clear() 

#     # GET ACTUAL CENTROIDS
#     for centroid in msg.poses:
#         # if centroid.position.x != 0.0:
#         centroids.append([centroid.position.x, centroid.position.y, centroid.position.z])  # NEW CENTROIDS
    
#     print('--------------------------------------------------------')
#     print('LAST: ', last_centroids)
#     print('NEW: ', centroids)


#     if not flag:
#         # AFTER FIRST TIME
#         print('FILTERS: ', *filters )
#         for nc in centroids:
#             distances = [
#                 math.sqrt( (nc[0] - lc[0])**2 + (nc[1] - lc[1])**2 + (nc[2] - lc[2])**2) for lc in last_centroids
#             ]
#             print('NEW: ', nc)
#             print('DISTANCES: ', distances)
#             print('MIN DISTANCE: ',min(distances))
#             print('CLOSER CENTROID FROM LC: ', last_centroids[distances.index(min(distances))])
#             if min(distances) > 2.0:
#                 print(nc,' ES UN NUEVO CENTROIDE')
#                 # filters[distances.index(min(distances))] = [ None, 0 ]
#                 filters[distances.index(min(distances))] = [ nc, Kalman_Filter() ]
#                 print('INDICE PARA CREAR INSTANCIA: ', distances.index(min(distances)))
#                 # filters[distances.index(min(distances))][0] = nc
#                 # filters[distances.index(min(distances))][1] = Kalman_Filter()
#                 print('NUEVA INSTANCIA CREADA: ', *filters)
#             else:
#                 print('ACTUAL Y ANTERIOR SON EL MISMO CENTROIDE: ', nc, last_centroids[distances.index(min(distances))])
#                 print('INDICE A ACTUALIZAR: ', distances.index(min(distances)))
#                 filters[distances.index(min(distances))][0] = nc
#                 print('ACTUALIZADO: ', *filters)
#             print('********** TERMINE **********')
#     else:
#         filters = [ [ c, Kalman_Filter() ] for c in centroids ]

#     flag = False


#     # CREATE A FILTER FOR EACH OBJECT DETECTED
#     # filters = [ [Kalman_Filter(), c] for c in centroids]
#     for f in filters:
#         f[1].ekf(f[0])                                                                               # FITERING


#     if filters:
#         id = 0
#         for f in filters:
#             if f[0][0] == 0.0 or f[0][1] == 0.0 or f[0][2] == 0.0:
#                 continue
#             else:
#                 marker = Marker()                                                                        # CREATE A MARKER FOR EACH OBJECT
#                 marker.header.frame_id = 'lidar_link'
#                 marker.ns = 'marker'
#                 marker.id = id
#                 marker.type = Marker.TEXT_VIEW_FACING
#                 marker.action = Marker.ADD
#                 marker.text = 'p[x, z] = ' + str(np.round(f[1].x[0],2)) + str(np.round(f[1].x[2], 2)) + '\n' 'v[x, z] = ' + str(np.round(f[1].x[3])) + str(np.round(f[1].x[5], 2))
#                 marker.pose.position.x = f[1].x[0]
#                 marker.pose.position.y = f[1].x[1]
#                 marker.pose.position.z = f[1].x[2]
#                 marker.scale.z = 1.5
#                 marker.color.r, marker.color.g, marker.color.b = [1.0, 1.0, 1.0]
#                 marker.color.a = 1.0
#                 marker.lifetime = rospy.Duration(0.5)
#                 marker_array.markers.append(marker)
#                 id +=1

#                 # POSE FOR EACH ESTIMATE
#                 object_pose = Pose()
#                 object_pose.position.x = f[1].x[0]                                                      # X - POSITION
#                 object_pose.position.y = f[1].x[1]                                                      # Y - POSITION
#                 object_pose.position.z = f[1].x[2]                                                      # Z - POSITION
#                 object_pose.orientation.x = f[1].x[3]                                                   # X - VELOCITY
#                 object_pose.orientation.y = f[1].x[4]                                                   # Y - VELOCITY
#                 object_pose.orientation.z = f[1].x[5]                                                   # Z - VELOCITY
                

#                 pose_array.poses.append(object_pose)