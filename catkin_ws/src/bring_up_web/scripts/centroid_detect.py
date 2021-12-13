#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray, Pose
import sensor_msgs.point_cloud2
import numpy as np
from sklearn.cluster import KMeans

# PUBLISHERS
pub_poses = None

def callback_object_detect(msg):
    global pub_poses

    if msg:
        points = sensor_msgs.point_cloud2.read_points(msg, skip_nans=True)                                          # GET DATA FROM POINT CLOUD
        dataset = []
        for point in points:
            if not point.__contains__(np.inf) and not point.__contains__(-np.inf):                                  # DELETE (-inf, inf)
                if (point[0] > 1.0 or point[0] < -1.0) and (point[1] > -1.5) and (point[2] > 2.0 or point[2] < -2.0):
                    dataset.append(list(point))                                                                     # DATASET TO CLUSTER

        # APPLY KMEANS
        kmeans = kmeans = KMeans(n_clusters=8, init='k-means++', n_init=10, max_iter=100, tol=0.01)     
        kmeans.fit_predict(dataset)
        centroids = kmeans.cluster_centers_                                                                         # GET CENTROIDS
        if centroids is not None:
            # MESSAGES
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
        
        
def main():

    global pub_poses

    print('Centroid Detect node...')
    rospy.init_node('object_detect')
    rate = rospy.Rate(10)

    # SUBSCRIBERRS
    rospy.Subscriber('/point_cloud', PointCloud2, callback_object_detect)

    # PUBLISHERS
    pub_poses = rospy.Publisher('/centroid_pose', PoseArray, queue_size=10)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInitException
