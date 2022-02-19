#!/usr/bin/env python3

import math
import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from kalman_filter import Kalman_Filter

# GLOBAL VARIABLES 
marker_array = MarkerArray()                                                                        # FILTER MARKERS
pose_array = PoseArray()                                                                            # FILTER POSES

def callback_object_pose(msg):

    global marker_array, pose_array
    centroids = []                                                                                   # ACTUAL CENTROIDS

    # GET ACTUAL CENTROIDS
    for centroid in msg.poses:
        if centroid.position.x != 0.0:
            centroids.append([centroid.position.x, centroid.position.y, centroid.position.z])       # POSE ARRAY TO LIST
    
    
    # CREATE A FILTER FOR EACH OBJECT DETECTED
    filters = [ [Kalman_Filter(), c] for c in centroids]
    for f in filters:
        f[0].ekf(f[1])                                                                               # FITERING


    if filters:
        id = 0
        for f in filters:
            marker = Marker()                                                                        # CREATE A MARKER FOR EACH OBJECT
            marker.header.frame_id = 'lidar_link'
            marker.ns = 'marker'
            marker.id = id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.text = 'p[x, z] = ' + str(np.round(f[0].x[0],2)) + str(np.round(f[0].x[2], 2)) + '\n' 'v[x, z] = ' + str(np.round(f[0].x[3])) + str(np.round(f[0].x[5], 2))
            marker.pose.position.x = f[0].x[0]
            marker.pose.position.y = f[0].x[1]
            marker.pose.position.z = f[0].x[2]
            marker.scale.z = 1.5
            marker.color.r, marker.color.g, marker.color.b = [1.0, 1.0, 1.0]
            marker.color.a = 1.0
            marker.lifetime = rospy.Duration(0.5)
            marker_array.markers.append(marker)
            id +=1

            # POSE FOR EACH ESTIMATE
            object_pose = Pose()
            object_pose.position.x = f[0].x[0]                                                      # X - POSITION
            object_pose.position.y = f[0].x[1]                                                      # Y - POSITION
            object_pose.position.z = f[0].x[2]                                                      # Z - POSITION
            object_pose.orientation.x = f[0].x[3]                                                   # X - VELOCITY
            object_pose.orientation.y = f[0].x[4]                                                   # Y - VELOCITY
            object_pose.orientation.z = f[0].x[5]                                                   # Z - VELOCITY
            

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
        pub_marker.publish(marker_array)                                                            # PUBLISH MARKERS
        pub_poses.publish(pose_array)                                                               # PUCBLISH ESTIMATES
        pose_array.poses.clear()
        marker_array.markers.clear()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException






