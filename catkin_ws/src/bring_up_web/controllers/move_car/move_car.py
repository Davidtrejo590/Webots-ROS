#!/usr/bin/env python3

"""move_car controller."""

# LIBRARIES
import math
from vehicle import Driver, Car
from controller import Camera, Keyboard
import os
import rospy
from std_msgs.msg import String

# CONSTANTS
TIME_STEP = 64
TRACK_FRONT = 1.7
WHEEL_BASE = 4.0
MAX_SPEED = 10.0

# INIT DRIVER
driver = Driver()

# INIT CAMERA
camera = Camera('camera')                                     # GET CAMERA FROM DEVICES
camera.enable(TIME_STEP)                                      

# INIT KEYBOARD
keyboard = Keyboard()
keyboard.enable(TIME_STEP)


# INIT ROS
pub = rospy.Publisher('chatter', String, queue_size=10)
rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(10) #10Hz

# ANGULO DE DIRECCION
right_angle = 0.0
left_angle = 0.0
driver.setSteeringAngle(0.0)

# CONTROL DE VELOCIDAD km/h
driver.setCruisingSpeed(0.0)

# CHANGE SPEED 
speed = 0.0
def change_speed():
  global speed
  # print('DRIVER STEP', driver.step())
  if( speed >= MAX_SPEED ):
    driver.setCruisingSpeed(MAX_SPEED)
  else:
    speed+= 0.5
    driver.setCruisingSpeed(speed)

  #print('CURRENT SPEED - km/h', round(driver.getCurrentSpeed(), 2))

# CHECK KEYBORARD
def check_keyboard():
  key = keyboard.getKey()
  # CALCULATE LEFT STEERING ANGLE
  if(key == 314):
    left_angle = math.atan(1 / (1/math.tan(driver.getSteeringAngle())) + TRACK_FRONT / (2 * WHEEL_BASE) )
    driver.setSteeringAngle(left_angle)
  # CALCULATE LEFT STEERING ANGLE
  elif (key == 316):
    right_angle = math.atan(1 / (1/math.tan(driver.getSteeringAngle())) - TRACK_FRONT / (2 * WHEEL_BASE) )
    driver.setSteeringAngle(right_angle)

# MAIN LOOP
while driver.step() != -1 and not rospy.is_shutdown():
  change_speed()
  check_keyboard()
  speed_str = "Current Speed %s" % round(driver.getCurrentSpeed(),2)
  rospy.loginfo(speed_str)
  pub.publish(speed_str)
  rate.sleep()
  pass
 