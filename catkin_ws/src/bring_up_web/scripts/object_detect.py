#!/usr/bin/env python3

""" 
    NODE TO GET THE POINT CLOUD FROM LIDAR
    AND IMPLEMENT THE KMEANS ALGORITHM WITH THE PURPOSE
    TO IDENTIFY OBSTACLES (CARS)
"""

# LIBRARIES
import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray, Pose
import sensor_msgs.point_cloud2
import matplotlib.pyplot as plt
import numpy as np
from random import uniform
import math
import copy

# GLOBAL VARIABLES
pose_array = PoseArray()

# OBEJCT DETECT CALLBACK
def callback_object_detect(msg):

    global pose_array

    if msg:
        points = sensor_msgs.point_cloud2.read_points(msg, skip_nans=True)          # GET EACH POINT IN THE POINT CLOUD
        current_centroids = []
        dataset = []
        
        for point in points:
            if not point.__contains__(np.inf) and not point.__contains__(-np.inf):
                if( (point[1] > -1.5) ):
                    dataset.append(list(point))                                     # DATASET TO APPLY KMEANS - (X, Y,  Z)

        # APPLY KMEANS
        current_centroids = kmeans(dataset)                                         # CUURENT CENTROIDS

        if current_centroids:
            for point in current_centroids:                                         # GET THE POSITION OF EACH CENTROID
                pose = Pose()
                pose.position.x = point[0]                                          # X COMPONENT 
                pose.position.y = point[1]                                          # Y COMPONENT
                pose.position.z = point[2]                                          # Z COMPONENT
            
                pose_array.poses.append(pose)                                       # POSE ARRAY <-- POSE FOR EACH CENTROID


def generate_centroids(dataset, k):
    min_x, min_y, min_z = np.amin(dataset, axis=0)                                  # GET MIN VALUE FOR X-AXIS AND Y-AXIS
    max_x, max_y, max_z = np.amax(dataset, axis=0)                                  # GET MAX VALUE FOR X-AXIS AND Y-AXIS
    centroids = [ 
        np.array([ 
            round(uniform(min_x, max_x), 3),
            round(uniform(min_y, max_y), 3),
            round(uniform(min_z, max_z), 3)  
            ]) for i in range(k)]

    
    return centroids                                                                # RETURN K-CENTROIDS

def calculate_centroids(point_cloud, centroids):
    new_centroids = []                                                              # CENTROITS RECALCULATED          
    clusters = [[] for c in centroids]                                              # CLUSTERS (GROUPS)

    # ASSIGN EACH POINT IN ITS CORRESPOND CLUSTER
    for p in point_cloud:
        distances = [ 
            math.sqrt((c[0] - p[0])**2 + (c[1] - p[1])**2 + (c[2] - p[2])**2 )      # EUCLEDIAN DISTANCE BETWEEN EACH POINT WITH EACH CENTROID
            for c in centroids ]       
        k_index = distances.index(min(distances))                                   # GET THE MINIMUM DISTANCES FOR EACH POINT
        clusters[k_index].append(p)                                                 # STORE IN THE CORRESPOND CLUSTER (GROUP)
    

    # RECOMPUTE CENTROIDS WITH THE MEAN OF EACH CLUSTER
    for cluster in clusters:
        if cluster:                                                                 # THE CURRENT CLUSTER HAS AT LEAST ONE POINT
            mean_k = np.mean(cluster, axis=0)                                       # COMPUTE THE MEAN OF EACH CLUSTER
            new_centroids.append(mean_k)                                            # ADD THE NEW CENTROID
        
    return new_centroids                                                            # RETURN A LIST WITH THE NEW CENTROIDS


# CALCUALTE THE DISTANCE BETWEEN EACH CENTROID D(OLD_CENTROID, NEW_CENTROID)
def compare_centroids(new_c, old_c):
    total_distance = 0.0

    for (oc, nc) in zip(old_c, new_c):                          
        distance = math.sqrt( (nc[0] - oc[0])**2 + (nc[1] - oc[1])**2 + (nc[2] - oc[2])**2 )    # EUCLEDIAN DISTANCE D(OLD_CENTROID, NEW_CENTROID)
        total_distance = total_distance + distance                                              # SUM OF EACH DISTANCE 
    
    return total_distance                                                                       # RETURN THE SUM OF THE DISTANCES

def kmeans(dataset):
    global initial_centroids
    k = 3                                                                                       # INIT NUMBER OF CLUSTERS (GROUPS)
    tol = 0.1                                                                                   # MINIMUN TOLERANCE FOR DISTANCE
    attempts = 0
    max_attempts = 100
    initial_centroids = generate_centroids(dataset, k)                                          # GENERATE K-CENTROIDS
    new_centroids = calculate_centroids(dataset, initial_centroids)                             # CALCULATE NEW CENTROIDS
    total_distance = compare_centroids(new_centroids, initial_centroids)                        # COMPUTE TOTAL DISTANCE BETWEEN INITAL & NEW CENTROIDS

    while total_distance > tol and attempts < max_attempts:
        centroids = copy.deepcopy(new_centroids)                                                # CENTROIDS <-- NEW CENTROIDS
        new_centroids = calculate_centroids(dataset, centroids)                                 # RECOMPUTE CENTROIDS
        total_distance = compare_centroids(new_centroids, centroids)                            # RECOMPUTE TOTAL DISTANCE
        attempts = attempts + 1
    # print(attempts)
    
    return new_centroids                                                                        # RETURN THE CURRENT CENTROIDS

# MAIN FUNCTION
def main():
    global pose_array
    global initial_centroids

    print('Object Detect Node...')
    rospy.init_node('object_detect')
    rate = rospy.Rate(10)

    # SUBSCRIBERS
    rospy.Subscriber('/point_cloud', PointCloud2, callback_object_detect)

    # MESSAGES 
    pose_array.header.stamp = rospy.Time.now()
    pose_array.header.frame_id = 'lidar_link'

    # PUBLISHERS
    pub_poses = rospy.Publisher('/object_pose', PoseArray, queue_size=10)
    
    # MAIN LOOP
    while not rospy.is_shutdown():
        pub_poses.publish(pose_array)                                 # PUBLISH CURRENT CENTROID (X, Z)
        pose_array.poses.clear()                                      # CLEAR POSE ARRAY
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except:
        rospy.ROSInitException
        pass
