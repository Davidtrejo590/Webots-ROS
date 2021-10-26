#!/usr/bin/env python3

""" ros_car_controller """

# LIBRARIES
import math
from sensor_msgs import msg
from vehicle import Driver, Car
from controller import Camera, Keyboard, Lidar, Gyro, GPS
import os
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Image, PointCloud2, PointField, NavSatFix, NavSatStatus, Imu

# CONSTANTS
TIME_STEP = 64
TRACK_FRONT = 1.7
WHEEL_BASE = 4.0

# GLOBAL VARIABLES
right_angle = 0.0
left_angle = 0.0


# INIT DRIVER
driver = Driver()
driver.setCruisingSpeed(0.0)                                  # SPEED CONTROL km/h - INITIAL SPEED
driver.setSteeringAngle(0.0)                                  # STEERING ANGLE - INITIAL ANGLE

# INIT CAMERA
camera = Camera('camera')                                     # GET CAMERA FROM DEVICES
camera.enable(TIME_STEP)    

# INIT LIDAR
lidar = Lidar('Sick LMS 291')
lidar.enable(TIME_STEP)
lidar.enablePointCloud()

# INIT GPS
gps = GPS('gps')
gps.enable(TIME_STEP)

# INIT GYRO
gyro = Gyro('gyro')
gyro.enable(TIME_STEP)

# INIT KEYBOARD
keyboard = Keyboard()
keyboard.enable(TIME_STEP)

# CHECK KEYBORARD
def check_keyboard():
  key = keyboard.getKey()
  # CALCULATE LEFT STEERING ANGLE
  if(key == Keyboard.LEFT):
    left_angle = math.atan(1 / (1/math.tan(driver.getSteeringAngle())) + TRACK_FRONT / (2 * WHEEL_BASE) )
    driver.setSteeringAngle(left_angle)
  # CALCULATE RIGHT STEERING ANGLE
  elif (key == Keyboard.RIGHT):
    right_angle = math.atan(1 / (1/math.tan(driver.getSteeringAngle())) - TRACK_FRONT / (2 * WHEEL_BASE) )
    driver.setSteeringAngle(right_angle)


# CRUISE SPEED CALLBACK
def callback_cruise_speed( msg ):
  speed = msg.data
  driver.setCruisingSpeed(speed)
  # print('Current Speed: ', speed_ros)  

# MAIN FUNCTION
def main():

  # INIT ROS
  print('INITIALIZING ROS CAR CONTROLLER NODE ...')
  rospy.init_node('ros_car_controller')
  # SUBSCRIPTIONS
  rospy.Subscriber('/pub_cruise_speed', Float64, callback_cruise_speed )
  rate = rospy.Rate(10)

  # IMAGE MESSAGE
  msg_image = Image()
  msg_image.height = camera.getHeight()
  msg_image.width = camera.getWidth()
  msg_image.is_bigendian = False
  msg_image.step = camera.getWidth() * 4
  msg_image.encoding = 'bgra8'

  # POINT CLOUD2 MESSAGE 
  msg_point_cloud = PointCloud2()
  msg_point_cloud.header.stamp = rospy.Time.now()
  msg_point_cloud.header.frame_id = 'lidar_cloud_point'  
  msg_point_cloud.height = 1
  msg_point_cloud.width = lidar.getNumberOfPoints()
  msg_point_cloud.point_step = 20
  msg_point_cloud.row_step = 20 * lidar.getNumberOfPoints()
  msg_point_cloud.is_dense = False
  msg_point_cloud.fields = [
    PointField(name = 'x', offset = 0, datatype = PointField.FLOAT32, count = 1),
    PointField(name = 'y', offset = 4, datatype = PointField.FLOAT32, count = 1),
    PointField(name = 'z', offset = 8, datatype = PointField.FLOAT32, count = 1),
  ]
  msg_point_cloud.is_bigendian = False

  # GPS MESSAGE
  # msg_gps = NavSatFix()
  # msg_gps.header.stamp = rospy.Time.now()
  # msg_gps.header.frame_id = 'frame_gps'
  # msg_gps.position_covariance = NavSatFix.COVARIANCE_TYPE_KNOWN
  # msg_gps.status.service = NavSatStatus.SERVICE_GPS
  # msg_gps.latitude = gps.getValues()[0]
  # msg_gps.longitude = gps.getValues()[1]
  # msg_gps.altitude = gps.getValues()[2]
  

  # GYRO MESSAGE
  msg_gyro = Imu()
  msg_gyro.header.stamp = rospy.Time.now()
  msg_gyro.header.frame_id = 'frame_gyro'
  

  # PUBLISHERS
  pub_camera_data = rospy.Publisher('pub_camera_data', Image, queue_size=10)
  pub_point_cloud = rospy.Publisher('pub_point_cloud', PointCloud2, queue_size=10)
  # pub_gps_data = rospy.Publisher('pub_gps_data', NavSatFix, queue_size=10)
  pub_imu_gyro = rospy.Publisher('pub_imu_gyro', Imu, queue_size=10)

  # MAIN LOOP
  while driver.step() != -1 and not rospy.is_shutdown():
    check_keyboard()                                                                    # CHECK KEYBOARD

    msg_image.data = camera.getImage()                                                  # GET IMAGE DATA FROM CAMERA
    msg_point_cloud.data = lidar.getPointCloud(data_type='buffer')                      # GET POINT CLOUD FROM LIDAR
    msg_gyro.angular_velocity.x = gyro.getValues()[0]                                   # GET X COMPONENT FROM GYRO
    msg_gyro.angular_velocity.y = gyro.getValues()[1]                                   # GET Y COMPONENT FROM GYRO
    msg_gyro.angular_velocity.z = gyro.getValues()[2]                                   # GET Z COMPONENT FROM GYRO

    pub_camera_data.publish(msg_image)                                                  # PUBLISHING IMAGE MESSAGE
    pub_point_cloud.publish(msg_point_cloud)                                            # PUBLISHING POINTCLOUD2 MESSAGE
    pub_imu_gyro.publish(msg_gyro)                                                      # PUBLISHING IMU MESSAGE
    # pub_gps_data.publish(msg_gps)
    
    rate.sleep()
    pass
 

if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass