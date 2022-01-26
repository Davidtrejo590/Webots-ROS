#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray, Pose
import sensor_msgs.point_cloud2
import numpy as np
from sklearn.cluster import KMeans, kmeans_plusplus
from scipy.cluster.vq import kmeans

# GLOBAL VARIABLES
initial_centroids = [
    [ -3.64229929,  -0.55005622,  -6.23897093],
    [  3.79652465,  -0.35820951,  -0.04962473],
    [  3.32333617,  -0.58985401, -14.47182993],
    [ -3.03360501,  -0.47881233,  22.09920644],
    [  4.61459438,  -0.60176526, -28.79273614],
    [ -3.74919798,  -0.60729853, -19.39469276],
    [ -3.68494749,  -0.58626728, -35.58342223],
    [ -4.00155359,  -0.52482279,  -5.27777102]
]

centroids = []

def callback_object_detect(msg):
    global centroids

    if msg:
        points = sensor_msgs.point_cloud2.read_points(msg, skip_nans=True)                                          # GET DATA FROM POINT CLOUD
        dataset = []
        for point in points:
            if not point.__contains__(np.inf) and not point.__contains__(-np.inf):                                  # DELETE (-inf, inf)
                if( (point[0] > 1.0 or point[0] < -1.0) and (point[1] > -0.75) ):
                    dataset.append(list(point))                                                                         # DATASET TO CLUSTER
            
        # APPLY KMEANS BY SKLEARN
        # kmeans = kmeans = KMeans(n_clusters=8, init='k-means++', n_init=10, max_iter=100, tol=0.01)     
        # kmeans.fit_predict(dataset)
        # centroids = kmeans.cluster_centers_                                                                         # GET CENTROIDS
        # APPLY KMEANS BY SCIPY
        centroids, dist = kmeans(dataset, 8)
        print(centroids)
        
def main():

    global pub_poses
    global centroids

    print('Centroid Detect node...')
    rospy.init_node('object_detect')
    rate = rospy.Rate(10)

    # SUBSCRIBERRS
    rospy.Subscriber('/point_cloud', PointCloud2, callback_object_detect)

    # PUBLISHERS
    pub_poses = rospy.Publisher('/object_pose', PoseArray, queue_size=10)


    while not rospy.is_shutdown():
        if centroids is not None:
            # MESSAGE
            pose_array = PoseArray()
            pose_array.header.stamp = rospy.Time.now()
            pose_array.header.frame_id = 'lidar_link'
            for point in centroids:
                pose = Pose()
                pose.position.x = point[0]
                pose.position.y = point[1]
                pose.position.z = point[2]

                pose_array.poses.append(pose)
            pub_poses.publish(pose_array)


if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInitException
