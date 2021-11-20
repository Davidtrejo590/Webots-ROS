#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray, Pose
import sensor_msgs.point_cloud2


def callback_object_detect(msg):

    pose = Pose()
    pose_array = PoseArray()
    pub_poses = rospy.Publisher('/object_pose', PoseArray, queue_size=10)

    for point in sensor_msgs.point_cloud2.read_points(msg, skip_nans=True):                             # GET POSITION OF OBSTACLE DETECTED
        pose.position.x = point[0]                                                                       # X COMPONENT
        pose.position.y = point[1]                                                                       # Y COMPONENT
        pose.position.z = point[2]                                                                       # Z COMPONENT

        pose_array.poses.append(pose)
        pub_poses.publish(pose_array)


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
