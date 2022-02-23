#!/usr/bin/env python3

import math
import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from kalman_filter import Kalman_Filter

# GLOBAL VARIABLES 
marker_array = MarkerArray()                            # FILTER MARKERS
pose_array = PoseArray()                                # FILTER POSES
centroids = []                                          # ACTUAL CENTROIDS
filters = None                                          # TO STORE OBJECTS OF KALMAN FILTER                    
flag = True                                             # FLAG TO CHECK THE FIRST TIME



def callback_object_pose(msg):

    global marker_array, pose_array, centroids, filters, flag

    last_centroids = centroids.copy()                   # LAST CENTROIDS ( LAST <-- NEW )
    centroids.clear() 

    # GET NEW CENTROIDS
    for centroid in msg.poses:
        # POSE ARRAY TO LIST
        centroids.append(
            [centroid.position.x, centroid.position.y, centroid.position.z]
        )  
    
    
    if not flag:                                                                    # AFTER FIRST TIME
        for nc in centroids:
            distances = [                                                           # COMPUTE DISTANCES
                math.sqrt( 
                    (nc[0] - lc[0])**2 + (nc[1] - lc[1])**2 + (nc[2] - lc[2])**2
                ) for lc in last_centroids
            ]
            # GET CLOSER CENTROID (c)
            min_distance = min(distances)
            closer_centroid = last_centroids[distances.index(min_distance)]
            closer_idx = distances.index(min_distance)
            if min_distance > 2.0:
                # NC IS A NEW CENTROID
                filters[closer_idx] = [ nc, Kalman_Filter() ]                       # CREATE A NEW OBJECT
            else:
                # NC AND C ARE THE SAME CENTROID
                filters[closer_idx][0] = nc                                         # UPDATE MEASURE
    else:                                                                           # IS THE FIRST TIME
        filters = [ [ c, Kalman_Filter() ] for c in centroids ]                     # CREATE N OBJECTS

    flag = False                                                                    


    # APPLY EKF TO FILTERS UPDATED
    for c,kalman_filter in filters:
        kalman_filter.ekf(c)


    # ADD MARKER TO EACH FILTER
    if filters:
        id = 0
        for f in filters:
            if f[0][0] == 0.0 or f[0][1] == 0.0 or f[0][2] == 0.0:
                continue
            else:
                marker = Marker()                                                   # CREATE A NEW MARKER FOR EACH OBJECT
                marker.header.frame_id = 'lidar_link'
                marker.ns = 'marker'
                marker.id = id
                marker.type = Marker.TEXT_VIEW_FACING
                marker.action = Marker.ADD
                marker.text = 'p[x, z] = ' + str(np.round(f[1].x[0],2)) + str(np.round(f[1].x[2], 2)) + '\n' 'v[x, z] = ' + str(np.round(f[1].x[3])) + str(np.round(f[1].x[5], 2))
                marker.pose.position.x = f[1].x[0]
                marker.pose.position.y = f[1].x[1]
                marker.pose.position.z = f[1].x[2]
                marker.scale.z = 1.5
                marker.color.r, marker.color.g, marker.color.b = [1.0, 1.0, 1.0]
                marker.color.a = 1.0
                marker.lifetime = rospy.Duration(0.5)
                marker_array.markers.append(marker)
                id +=1

                # POSE FOR EACH ESTIMATE
                object_pose = Pose()
                object_pose.position.x = f[1].x[0]                                  # X - POSITION
                object_pose.position.y = f[1].x[1]                                  # Y - POSITION
                object_pose.position.z = f[1].x[2]                                  # Z - POSITION
                object_pose.orientation.x = f[1].x[3]                               # X - VELOCITY
                object_pose.orientation.y = f[1].x[4]                               # Y - VELOCITY
                object_pose.orientation.z = f[1].x[5]                               # Z - VELOCITY
                

                pose_array.poses.append(object_pose)

    
    


def main():

    global marker_array, pose_array

    print('Object Tracking Node...')
    rospy.init_node('object_tracking')
    rate = rospy.Rate(10)

    # SUBSCRIBERS
    rospy.Subscriber('/object_pose', PoseArray, callback_object_pose)

    # PUBLISHER
    pub_marker = rospy.Publisher('/marker', MarkerArray, queue_size=10)
    pub_poses = rospy.Publisher('/filter_pose', PoseArray, queue_size=10)
    

    while not rospy.is_shutdown():
        pub_marker.publish(marker_array)                                      # PUBLISH MARKERS
        pub_poses.publish(pose_array)                                         # PUCBLISH ESTIMATES
        pose_array.poses.clear()
        marker_array.markers.clear()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException


# PESUDOCÓDIGO


# PARA TODO centroide nuevo CN:
#     Obtener el centroide más cercano C de los centroides previos. 
#     Si la distancia entre C y CN es mayor que un umbral (1 o dos metros) 
#         entonces CN es un nuevo centroide
#         Agregar CN al conjunto de centroides con un nuevo ID
#         Crear un nuevo filtro de kalman con el ID anterior
#     Si no
#         entonces CN es el mismo que C y no es necesario crear un nuevo filtro de Kalman-
#         Actualizar el EKF correspondiente a C con las coordenadas de CN


