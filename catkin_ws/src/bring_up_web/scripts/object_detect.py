#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray, Pose
import sensor_msgs.point_cloud2
import matplotlib.pyplot as plt


x = []
z = []

def callback_object_detect(msg):
    pose_array = PoseArray()
    pose_array.header.stamp = rospy.Time.now()
    pose_array.header.frame_id = 'lidar_link'

    # PUBLISHERS
    pub_poses = rospy.Publisher('/object_pose', PoseArray, queue_size=10)   

    points = sensor_msgs.point_cloud2.read_points(msg, skip_nans=True)
    for point in points:
        pose = Pose()
        pose.position.x = point[0]                                                                       # X COMPONENT
        pose.position.y = point[1]                                                                       # Y COMPONENT
        pose.position.z = point[2]                                                                       # Z COMPONENT
        x.append(round(point[0], 3))
        z.append(round(point[2], 3))

        pose_array.poses.append(pose)
        pub_poses.publish(pose_array)
    
    # plt.scatter(x, z)
    # plt.xlabel('X')
    # plt.ylabel('Z')
    # plt.title('Points')
    # plt.show()
    # print('Start',  x, '\n')
    # x.clear()
    # z.clear()
    

def main():
    print('Object detect node...')
    rospy.init_node('object_detect')
    rate = rospy.Rate(10)

    # SUBSCRIBERS
    rospy.Subscriber('/point_cloud', PointCloud2, callback_object_detect)
    
    while not rospy.is_shutdown():
        rate.sleep()
        pass



if __name__ == "__main__":
    try:
        main()
    except:
        rospy.ROSInitException
        pass
