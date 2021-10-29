#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

bridge = CvBridge()

def callback_camera_data(msg):
    cv_image = bridge.imgmsg_to_cv2(msg)
    

def main():
    print('In camera node...')
    rospy.init_node('camera_data')
    rospy.Subscriber('/camera/rgb/raw', Image, callback_camera_data)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInitException
        pass