#!/usr/bin/env python3

import rospy
from sensor_msgs import msg
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray, Pose
import sensor_msgs.point_cloud2
import matplotlib.pyplot as plt


x, y, z = [[], [], []]

def callback_object_detect(msg):
    global x, y, z

    for point in sensor_msgs.point_cloud2.read_points(msg, skip_nans=True):         # GET POSITION OF EACH POINT IN THE POINT CLOUD
        x.append(point[0])                                                          # X COMPONENT                
        y.append(point[1])                                                          # Y COMPONENT 
        z.append(-1 * point[2])                                                     # Z COMPONENT 


def main():

    print('Object detect node...')
    rospy.init_node('object_detect')
    rate = rospy.Rate(10)

    # SUBSCRIBERS
    rospy.Subscriber('/point_cloud', PointCloud2, callback_object_detect)

    # MESSAGES
    msg_pose = Pose()

    # PUBLISHERS
    pub_pose = rospy.Publisher('/object_pose', Pose, queue_size=10)
    
    while not rospy.is_shutdown():

        if len(x) > 0 and len(z) > 0 and len(y) > 0:
            for point in range(len(x)):
                msg_pose.position.x = x[point]
                msg_pose.position.y = y[point]
                msg_pose.position.z = z[point]
                # plt.scatter(x, z)
                # plt.xlabel('X')
                # plt.ylabel('Z')
                # plt.title('Points')
                # plt.show()
                pub_pose.publish(msg_pose)

        rate.sleep()
        pass

if __name__ == "__main__":
    try:
        main()
    except:
        rospy.ROSInterruptException
        pass
