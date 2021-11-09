#!/usr/bin/env python2

""" PROCESS RGB IMAGE FROM ROS TO GET LEFT LINE OF LANE  """

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
import cv2
from cv_bridge import CvBridge
import numpy as np
import matplotlib.pyplot as plt

# GLOBAL VARIABLES
left_lane = []
right_lane = []


# GET EDGES OF AN IMAGE
def canny(image):
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)                              # CONVERT TO GRAY SCALE
    blur = cv2.GaussianBlur(gray, (5, 5), 0)                                    # APPLY BLUR FILTER TO REDUCE NOISE
    canny = cv2.Canny(blur, 50, 150)                                            # DETECT EDGES WITH CANNY

    return canny

# CALCULATE INTEREST REGION
def region_of_interest(image):
    height = image.shape[0]
    triangle = np.array([[
        (0, 450),                                                               # FIRST POINT, LEFT-BOTTOM
        (640, 450),                                                             # SECOND POINT, RIGHT-BOTTOM
        (320, 250)                                                              # THIRD POINT, CENTER
    ]])
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, triangle, 255)
    masked_img = cv2.bitwise_and(image, mask)

    return masked_img

# DISPLAY LINES AS AN IMAGE
def display_lines(image, lines):
    line_img = np.zeros_like(image)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)                                    # GET 2 POINTS OF EACH LINE
            cv2.line(line_img, (x1, y1), (x2, y2), [255, 0, 0], 10)             # DRAW A LINE 
    return line_img                                                             # RETURN AN IMAGE WITH LINES


def avg_slope_intercept(image, lines):
    left_fit = []                                                               # LEFT LINES
    right_fit = []                                                              # RIGHT LINES
    
    for line in lines:
        x1, y1, x2, y2 = line.reshape(4)                                        # GET 2 POINTS OF EACH LINE
        # print('Puntos:', x1, y1, x2, y2)
        parameters = np.polyfit((x1, x2), (y1, y2), 1)                          # CALCULATE SLOPE(m) AND INTERCEPT(b) WITH y = mx + b
        # print('Pendiente[m] y Ordenada al origen[b]: ', parameters)
        slope = parameters[0]                                                   # SLOPE(m)
        intercept = parameters[1]                                               # INTERCEPT(b)
        if slope < 0:
            left_fit.append((slope, intercept))                                 # WITH (-m, b) 
        else:
            right_fit.append((slope, intercept))                                # WITH (+m, b)
    # print('Left Lines: ', left_fit)
    # print('Right Lines:', right_fit)
    left_fit_avg = np.average(left_fit, axis=0)                                 # LEFT LINES AVG [m, b]
    right_fit_avg = np.average(right_fit, axis=0)                               # RIGHT LINES AVG [m ,b]
    # print('Left lines avg: ', left_fit_avg)
    # print('Right lines avg: ', right_fit_avg)
    left_line = make_coordinates(image, left_fit_avg)                           # LEFT LINE COORDINATES
    right_line = make_coordinates(image, right_fit_avg)                         # RIGHT LINE COORDINATES
    # print('Left Coordinates: ', left_line)
    # print('Right Coordinates: ', right_line)
    return np.array([left_line, right_line])                                    # RETURN LEFT AND RIGHT COORDINATES

# MAKE COORDINATES OF THE LINES
def make_coordinates(image, line_parameters):
    slope, intercept = line_parameters                                          # SLOPE = m, INTERCEPT = b
    # y1 = image.shape[0]                                                       # IMG HEIGHT (480)
    y1 = 480                                           
    y2 = int((y1*(0.6)))
    x1 = int((y1 - intercept)/slope)                                            # X1 = (Y1 - b)/m
    x2 = int((y2 - intercept)/slope)                                            # X1 = (Y1 - b)/m
    # print(x1, y1, x2, y2)
    return np.array([x1, y1, x2, y2])                                           # RETURN TWO POINTS AS A LINE (X1, Y1) - (X2, Y2)



def callback_lane_detect(msg):

    global left_lane
    global right_lane

    bridge = CvBridge()
    cv2_img = bridge.imgmsg_to_cv2(msg, 'bgr8')                                 # ROS IMAGE TO CV2 IMAGE ( RGB )
    lane_img = np.copy(cv2_img)                                                 # COPY OF ''cv_img'
    canny_img = canny(lane_img)                                                 # CANNY IMAGE ( EDGES )
    cropped_img = region_of_interest(canny_img)                                 # COPPRED IMAGE ( ONLY INTEREST REGION ) 
    lines = cv2.HoughLinesP(                                                    # HOUGHP ( LANE LINES )
            cropped_img,
            2,
            np.pi/180,
            100,
            np.array([]),
            minLineLength=40,
            maxLineGap=50
        )
    # print(lines)
    if(lines is not None):
        avg_lines = avg_slope_intercept(lane_img, lines)                            # LEFT AND RIGHT LINES AS COORDINATES
        # print('AVG LINES: ', avg_lines.reshape(2,4))
        left_lane, right_lane = avg_lines.reshape(2,4)
        line_img = display_lines(lane_img, avg_lines)                               # DISPLAY LINES IN A IMAGE
        combo_img = cv2.addWeighted(lane_img, 0.8, line_img, 1, 1)                  # LANE_IMG + LINES
        cv2.imshow('Result', combo_img)                                             # DISPLAY IMAGE 
        cv2.waitKey(33)
    else:
        cv2.imshow('Result', lane_img)                                             # DISPLAY IMAGE 
        cv2.waitKey(33)
        # print('BUG', lines, left_lane, right_lane)
        left_lane = []
        right_lane = []


    # SHOW GRAPH IMAGE
    # plt.imshow(combo_img)
    # plt.show()


def main():
    print('Lane Detect node...')
    rospy.init_node('left_line')
    rate = rospy.Rate(10)
    rospy.Subscriber('/camera/rgb/raw', Image, callback_lane_detect)

     # MESSAGES
    msg_left_lane = Float64MultiArray()
    msg_right_lane = Float64MultiArray()

    # PUBLISHERS
    pub_left_lane = rospy.Publisher('/left_lane', Float64MultiArray, queue_size=10)
    pub_right_lane = rospy.Publisher('/right_lane', Float64MultiArray, queue_size=10)


    while not rospy.is_shutdown():
        # LEFT AND RIGHT LANE
        msg_left_lane.data = left_lane
        msg_right_lane.data = right_lane

        # PUBLISH
        pub_left_lane.publish(msg_left_lane)
        pub_right_lane.publish(msg_right_lane)
        
        rate.sleep()
        pass

    # rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except:
        rospy.ROSInitException
        pass