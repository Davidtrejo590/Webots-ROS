#!/usr/bin/env python2

""" PROCESS RGB IMAGE FROM ROS TO GET LEFT LINE OF LANE  """

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

def canny(image):
    # CONVERT TO GRAY SCALE
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    # APPLY BLUR FILTER TO REDUCE NOISE
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    # DETECT EDGES WITH CANNY
    canny = cv2.Canny(blur, 50, 150)

    return canny

def region_of_interest(image):
    height = image.shape[0]
    triangle = np.array([[
        (0, 450),                                   # FIRST POINT, LEFT-BOTTOM
        (640, 450),                                 # SECOND POINT, RIGHT-BOTTOM
        (320, 250)                                  # THIRD POINT, CENTER
    ]])
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, triangle, 255)
    masked_img = cv2.bitwise_and(image, mask)
    return masked_img

def display_lines(image, lines):
    line_img = np.zeros_like(image)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            # print(x1, y1, x2, y2)
            cv2.line(line_img, (x1, y1), (x2, y2), [255, 0, 0], 10)
    return line_img

def avg_slope_intercept(image, lines):
    left_fit = []
    right_fit = []
    for line in lines:
        x1, y1, x2, y2 = line.reshape(4)
        # print('Puntos:', x1, y1, x2, y2)
        parameters = np.polyfit((x1, x2), (y1, y2), 1)                                      # y = mx + b
        # print('Pendiente[m] y Ordenada al origen[b]: ', parameters)
        slope = parameters[0]
        intercept = parameters[1]
        if slope < 0:
            left_fit.append((slope, intercept))
        else:
            right_fit.append((slope, intercept))
    # print('Left Lines: ', left_fit)
    # print('Right Lines:', right_fit)
    left_fit_avg = np.average(left_fit, axis=0)                                         # PROMEDIO DE LAS LINEAS IZQUIERDAS [m, b]
    right_fit_avg = np.average(right_fit, axis=0)                                       # PROMEDIO DE LAS LINEAS DERECHAS [m ,b]
    # print('Left lines avg: ', left_fit_avg)
    # print('Right lines avg: ', right_fit_avg)
    left_line = make_coordinates(image, left_fit_avg)
    right_line = make_coordinates(image, right_fit_avg)
    # print('Left Coordinates: ', left_line)
    # print('Right Coordinates: ', right_line)
    return np.array([left_line, right_line])

def make_coordinates(image, line_parameters):
    slope, intercept = line_parameters
    # y1 = image.shape[0]                                                 # img height
    y1 = 450
    y2 = int((y1*(0.6)))
    x1 = int((y1 - intercept)/slope)
    x2 = int((y2 - intercept)/slope)
    print(x1, y1, x2, y2)
    return np.array([x1, y1, x2, y2])



def callback_left_line(msg):

    bridge = CvBridge()
    cv2_img = bridge.imgmsg_to_cv2(msg, 'bgr8')                     # ROS IMAGE TO CV2 IMAGE ( RGB )
    lane_img = np.copy(cv2_img)
    # CANNY IMAGE
    canny_img = canny(lane_img)
    # COPPRED IMAGE
    cropped_img = region_of_interest(canny_img)
    lines = cv2.HoughLinesP(cropped_img, 2, np.pi/180, 100,
                        np.array([]), minLineLength=40, maxLineGap=50)
    # print('Lines:', lines)
    avg_lines = avg_slope_intercept(lane_img, lines)
    # print('avg lines: ', avg_lines)
    line_img = display_lines(lane_img, avg_lines)
    combo_img = cv2.addWeighted(lane_img, 0.8, line_img, 1, 1)


    cv2.imshow('Result', combo_img)                                 # DISPLAY IMAGE 
    cv2.waitKey(33)


def main():
    print('Lane Detect node...')
    rospy.init_node('left_line')
    rospy.Subscriber('/camera/rgb/raw', Image, callback_left_line)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except:
        rospy.ROSInitException
        pass