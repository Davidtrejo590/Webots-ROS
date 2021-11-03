#!/usr/bin/env python2

""" camera_data for proccess image data from ROS message """

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
import cv2
from cv_bridge import CvBridge
import numpy
import math

# CALCULATE REGION OF INTEREST 
def region_of_interest(img_edges):
    height = img_edges.shape[0]
    width = img_edges.shape[1]
    mask = numpy.zeros_like(img_edges)
    triangle = numpy.array([[
        (-850, height),                                                         # BOTTOM LEFT TRIANGLE POINT
        (330, 265),                                                             # MIDDLE TRIANGLE POINT
        (850, height)                                                           # BOTTOM RIGHT TTRIANGLE POINT
        ]], numpy.int32)
    cv2.fillPoly(mask, triangle, 255)
    img_masked = cv2.bitwise_and(img_edges, mask)                               # RETURN ONLY THE INTEREST AREA - TRIANGLE OF INTEREST

    return img_masked

# CALCULATE HOUGH LINES
def hough_lines(coppred_img):
    lines = cv2.HoughLinesP(coppred_img, rho=1.0, theta= math.pi/180, threshold=60, lines=None, minLineLength=175, maxLineGap=150)
    return lines

# DISPLAY LINES IN BGR IMAGE
def display_lines(bgr_img, avg_lines):
    # DRAWING LINES
    lines_img = numpy.zeros((bgr_img.shape[0], bgr_img.shape[1], 3), dtype=numpy.uint8)
    thickness_lines = 4
    line_color = [255, 0, 0]                                                     # LINE COLOR - BLUE
    dot_color = [0, 0, 255]                                                      # DOT COLOR - RED
    dot_size = 5

    for line in avg_lines:
        for x1, y1, x2, y2, in line:
            cv2.line(lines_img, (x1, y1), (x2, y2), line_color, thickness_lines)
            cv2.circle(lines_img, (x1, y1), dot_size, dot_color, -1)
            cv2.circle(lines_img, (x2, y2), dot_size, dot_color, -1)

    return lines_img

# ADD WEIGHTED TO BGR IMAGE
def add_weighted(bgr_img, lines_img):
    try:
        return cv2.addWeighted(src1=bgr_img, alpha=0.8, src2=lines_img, beta=1.0, gamma=0.0)    # MAKE LINES MORE VISIBLES
    except:
        pass

def calculate_inclination_angle_list(lines):
    try:
        if(len(lines) == 0):
            print('No lines found')
            pass
        else:
            slope_list = []
            for line in lines:
                x1 = line[0][0]
                y1 = line[0][1]
                x2 = line[0][2]
                y2 = line[0][3]

                slope = (float(y2)-float(y1) / float(x2)-float(x1))
                slope = (math.atan(slope)*180) / math.pi
                slope_list.append(slope)
        return slope_list
    except:
        pass


# CALLBACK CAMERA DATA
def callback_camera_data(msg):

    lower_white = numpy.array([177, 177, 177])                                  # LOWER THRESHOLD
    upper_white = numpy.array([255, 255, 255])                                  # UPPER THRESHOLD
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))                  # OPENCV KERNEL

    # CV BRIDGE
    bridge = CvBridge()

    cv2_img = bridge.imgmsg_to_cv2(msg, 'bgr8')                                 # ROS IMAGE TO CV2 IMAGE ( RGB )
    # cv2.imshow('Original Image', cv2_image)                                   # SHOW ORGININAL IMAGE IN RGB
    bin_img = cv2.inRange(cv2_img, lower_white, upper_white)                    # SEGMENTATION WHITE LANES
    # cv2.imshow('Binary Image', bin_img)                                       # SHOW BINARY IMAGE
    noise_filter_img = cv2.morphologyEx(bin_img, cv2.MORPH_OPEN, kernel)        # APPLY FILTER TO REDUCE NOISE
    # cv2.imshow('Binary Image filtered', noise_filter_img)                     # SHOW BINARY FILTERED
    edge_img = cv2.Canny(noise_filter_img, 100, 200)                            # GET IMAGE EDGES
    # cv2.imshow('Edge Image', edge_img)                                        # SHOW EDGE IMAGE
    coppred_img = region_of_interest(edge_img)                                  # GET COPPRED IMAGE
    # cv2.imshow('Interest Region', coppred_img)                                # SHOW AREA OF INTEREST
    lines = hough_lines(coppred_img)                                            # CALCULATE LINES WITH HOUGHP TRANSFORM
    lines_img = display_lines(cv2_img, lines)                                   # GET ORIGINAL IMAGE WITH LINES
    overlayed_img = add_weighted(cv2_img, lines_img)                            # ORIGINAL IMAGE WITH LINES OVERLAYED
    cv2.imshow('Image Overlayed', overlayed_img)                                # SHOW ORIGINAL IMAGE WITH LINES OVERLAYED

    cv2.waitKey(33)                                                             # WAIT FOR EACH FRAME

    # MESSAGES
    msg_speed = Float64()                                                       
    msg_angle = Float64()

    # PUBLISHERS
    pub_speed = rospy.Publisher('/goal_cruise_speed', Float64, queue_size=10)
    pub_angle = rospy.Publisher('/goal_steering_angle', Float64, queue_size=10)

    angle_list = calculate_inclination_angle_list(lines)                        # CALCULATE SLOPE AND ANGLE INCLINATION OF THE LINES

    # CHECK LINES TO DETECT LANES
    if(isinstance(angle_list, list)) == True:
        upper_angle_list = []

        for angle in angle_list:
            if angle < 0:
                upper_angle_list.append(angle)
        
        upper_angle_list.sort()                                                 # SORT ANGLES
        
        if(abs(upper_angle_list[0]) > 36.5):
            print('The car is staright')
            msg_speed.data = 10.0
            # msg_angle.data = 90.0
        else:
            print('The car is not staright')
            # angle_error = 39 - abs(upper_angle_list[0])
            msg_speed.data = 0.0
            # msg_angle.data = 90 - (int(angle_error)*90) / 39


        pub_speed.publish(msg_speed)                                            # PUBLISH SPEED
        # pub_angle.publish(msg_angle)

    else:
        pass

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
