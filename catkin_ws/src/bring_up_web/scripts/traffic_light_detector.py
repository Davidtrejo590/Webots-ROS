#!/usr/bin/env python2

import numpy
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()
publisher = rospy.Publisher('traffic_light', String, queue_size=10)

def process_image(raw_image):
  image = bridge.imgmsg_to_cv2(raw_image, 'bgr8')
  
  height = image.shape[0]
  width = image.shape[1]
  mask = numpy.zeros_like(image)
  polygon = numpy.array([[(0,height),(width,height),(width,410),(400,255),(400,50),(175,50),(175,260),(0,310)]])
  cv2.fillPoly(mask, polygon, (255,255,255))
  cropped_image = cv2.bitwise_and(image, mask)
  
  lower_green = numpy.array([85,165,25])
  upper_green = numpy.array([160,215,100])
  lower_yellow = numpy.array([55,155,190])
  upper_yellow = numpy.array([145,195,220])
  lower_red = numpy.array([40,35,195])
  upper_red = numpy.array([135,125,225])
  
  binary_image_g = cv2.inRange(cropped_image, lower_green, upper_green)
  binary_image_y = cv2.inRange(cropped_image, lower_yellow, upper_yellow)
  binary_image_r = cv2.inRange(cropped_image, lower_red, upper_red)
  
  g_pixels = cv2.countNonZero(binary_image_g)
  y_pixels = cv2.countNonZero(binary_image_y)
  r_pixels = cv2.countNonZero(binary_image_r)
  
  kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2,2))
  if g_pixels > y_pixels and g_pixels > r_pixels:
    filtered_image = cv2.morphologyEx(binary_image_g, cv2.MORPH_OPEN, kernel)
    light = 'Green'
  elif y_pixels > g_pixels and y_pixels > r_pixels:
    filtered_image = cv2.morphologyEx(binary_image_y, cv2.MORPH_OPEN, kernel)
    light = 'Yellow'
  else:
    filtered_image = cv2.morphologyEx(binary_image_r, cv2.MORPH_OPEN, kernel)
    light = 'Red'
    
  filtered_image_color = cv2.bitwise_and(image, image, mask=filtered_image)
  
  gray_image = cv2.cvtColor(filtered_image_color, cv2.COLOR_BGR2GRAY)
  gray_image = cv2.medianBlur(gray_image, 3)

  rows = gray_image.shape[0]
  circles = cv2.HoughCircles(gray_image, cv2.HOUGH_GRADIENT, 1, rows*0.75, param1=600, param2=1, minRadius=5, maxRadius=8)
  if circles is not None:
    circles = numpy.uint16(numpy.around(circles))
    for i in circles[0,:]:
      center = (i[0], i[1])
      radius = i[2]
      cv2.circle(image, center, radius, (255,255,255))
    publisher.publish(String(data=light))
  else:
    publisher.publish(String(data='None'))
  cv2.imshow('Processed Image', image)
  cv2.waitKey(33)

def main():
  rospy.init_node('traffic_light_detector')
  rospy.Subscriber('/camera/rgb/raw',Image,process_image)
  rospy.spin()

if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass
