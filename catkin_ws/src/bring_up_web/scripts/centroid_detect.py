#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray, Pose
import sensor_msgs.point_cloud2
import numpy as np
from sklearn.cluster import KMeans, kmeans_plusplus
from scipy.cluster.vq import kmeans





# GLOBAL VARIABLES
centroids = []

def callback_object_detect(msg):
    global centroids

    if msg:
        points = sensor_msgs.point_cloud2.read_points(msg, skip_nans=True)                                          # GET DATA FROM POINT CLOUD
        dataset = []
        for point in points:
            if not point.__contains__(np.inf) and not point.__contains__(-np.inf):                                  # DELETE (-inf, inf)
                if (point[0] > 0.5 or point[0] < -0.5) and (point[1] > -1.5) and (point[2] > 2.5 or point[2] < -2.5):    
                    dataset.append(list(point))                                                                     # DATASET TO CLUSTER
            
        # APPLY KMEANS
        # kmeans = kmeans = KMeans(n_clusters=8, init='k-means++', n_init=10, max_iter=100, tol=0.01)     
        # kmeans.fit_predict(dataset)
        # centroids = kmeans.cluster_centers_                                                                         # GET CENTROIDS
        initial_centroids = [ [ 3.184,  0.   , -2.778], [-2.457,  0.   , -7.841], [ -4.469,   0.   , -10.935], [ -2.123,   0.   , -13.339], [-5.538,  0.   , -3.623], [  2.546,   0.   , -12.516], [-3.366,  0.   , -9.381], [-1.63 ,  0.   , -7.704] ]
        centroids, dist = kmeans(dataset, initial_centroids)


        
        
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
