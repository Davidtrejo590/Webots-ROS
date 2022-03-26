#!/usr/bin/env python3

""" 
    NODE TO GET THE POINT CLOUD FROM LIDAR
    AND APPLY THE KMEANS ALGORITHM BY 'SCRIPY' LIBRARY 
    WITH THE PURPOSE TO IDENTIFY OBSTACLES (CARS)
"""


import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray, Pose
import sensor_msgs.point_cloud2
import numpy as np
from scipy.cluster.vq import kmeans

centroids = []                                  # CENTROIDS CALCULATED BY KMEANS


# CALLBACK POINT CLOUD
def callback_point_cloud(msg):
    global centroids

    if msg:
        dataset = []
        points = sensor_msgs.point_cloud2.read_points(msg, skip_nans=True)          # GET DATA FROM POINT CLOUD
        for point in points:
            if not point.__contains__(np.inf) and not point.__contains__(-np.inf):  # DELETE (-inf, inf)
                if point[1] > -1.5 :                                                # FILTER DATA
                    dataset.append(list(point))                                     # DATASET TO CLUSTERING

        # APPLY KMEANS BY SCIPY
        centroids, dist = kmeans(dataset, 4)
        
# MAIN FUNCTION
def main():

    global centroids

    print('Object Detect Node...')
    rospy.init_node('object_detect')
    rate = rospy.Rate(10)

    # SUBSCRIBERRS
    rospy.Subscriber('/point_cloud', PointCloud2, callback_point_cloud)

    # PUBLISHERS
    pub_poses = rospy.Publisher('/object_pose', PoseArray, queue_size=10)

    while not rospy.is_shutdown():
        if centroids is not None:
            # POSE ARRAY MESSAGE ( CENTROIDS )
            pose_array = PoseArray()
            pose_array.header.stamp = rospy.Time.now()
            pose_array.header.frame_id = 'lidar_link'
            for point in centroids:
                pose = Pose()
                pose.position.x = point[0]
                pose.position.y = point[1]
                pose.position.z = point[2]

                pose_array.poses.append(pose)
            pub_poses.publish(pose_array)                       # PUBLISH MESSAGE

        rate.sleep()
        

if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInitException
