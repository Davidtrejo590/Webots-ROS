#!/usr/bin/env python3

""" 
    NODE TO OBTAIN THE CENTROIDS CALCULATED USING THE K MEANS ALGORITHM, 
    APPLY A TIE ALGORITHM TO THE CENTROIDS FOR OBJECT TRACKING WITH THE EKF CLASS
"""

# LIBRARIES
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
first_time = True                                       # FLAG TO CHECK THE FIRST TIME


# ADD LABELS TO OBJECTS ESTIMATES
def add_labels( filters ):
    object_markers = MarkerArray()                      # LABELS FOR ESTIMATES
    object_poses = PoseArray()                          # POSITION & VELOCITIES ESTIMATED

    id = 0
    for c, kalman_filter in filters:
        if c[0] == 0.0 or c[2] == 0.0:                  # CHECK CENTROIDS IN 0.0
            continue
        else:
            marker = Marker()                           # CREATE A NEW MARKER FOR EACH OBJECT
            marker.header.frame_id = 'lidar_link'
            marker.ns = 'marker'
            marker.id = id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.text = 'p[x, z] = ' + str(np.round(kalman_filter.x[0],2)) + str(np.round(kalman_filter.x[1], 2)) + '\n' 'v[x, z] = ' + str(np.round(kalman_filter.x[2], 2)) + str(np.round(kalman_filter.x[3], 2))
            marker.pose.position.x = kalman_filter.x[0]
            marker.pose.position.y = 0.0
            marker.pose.position.z = kalman_filter.x[1]
            marker.scale.z = 1.5
            marker.color.r, marker.color.g, marker.color.b = [1.0, 1.0, 1.0]
            marker.color.a = 1.0
            marker.lifetime = rospy.Duration(0.5)
            object_markers.markers.append(marker)
            id +=1

            object_pose = Pose()                                    # CREATE A NEW POSE FOR EACH ESTIMATE
            object_pose.position.x = kalman_filter.x[0]             # X - POSITION
            object_pose.position.y = 0.0                            # Y - POSITION
            object_pose.position.z = kalman_filter.x[1]             # Z - POSITION
            object_pose.orientation.x = kalman_filter.x[2]          # X - VELOCITY
            object_pose.orientation.y = 0.0                         # Y - VELOCITY
            object_pose.orientation.z = kalman_filter.x[3]          # Z - VELOCITY
            object_poses.poses.append(object_pose)

    return [object_markers, object_poses]                           # RETURN LABELS AND POSES ESTIMATES INTO AN ARRAY



def callback_object_pose(msg):

    global marker_array, pose_array, centroids, filters, first_time

    last_centroids = centroids.copy()                                               # LAST CENTROIDS ( LAST <-- NEW )
    centroids.clear() 

    # GET NEW CENTROIDS
    for centroid in msg.poses:
        centroids.append(
            [centroid.position.x, centroid.position.y, centroid.position.z]         # POSE ARRAY TO LIST
        )  
    
    
    if not first_time:                                                              # AFTER FIRST TIME
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

    first_time = False                                                              # FOR NEXT ITMES                                                    


    # APPLY EKF TO FILTERS UPDATED
    for c,kalman_filter in filters:
        kalman_filter.ekf(c)

    # ADD LABELS AND GET AN ARRAY WITH POSITION & VELOCITIES TO PUBLISH FOR EACH FILTER
    if filters:
        marker_array, pose_array = add_labels(filters)
        

# MAIN FUNCTION
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
        pub_marker.publish(marker_array)                         # PUBLISH MARKERS
        pub_poses.publish(pose_array)                            # PUCBLISH ESTIMATES
        pose_array.poses.clear()
        marker_array.markers.clear()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException