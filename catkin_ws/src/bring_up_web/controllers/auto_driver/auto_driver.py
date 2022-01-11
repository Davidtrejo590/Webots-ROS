#!/usr/bin/env python3

import math
from vehicle import Driver
from controller import Camera, Keyboard, Lidar, Gyro, GPS
import rospy
from std_msgs.msg import Float64, String
from sensor_msgs.msg import Image, PointCloud2, PointField, NavSatFix, NavSatStatus, Imu

TIME_STEP = 64
TRACK_FRONT = 1.7
WHEEL_BASE = 4.0

cruise_speed = 0
steering_angle = 0
traffic_light = 'None'

driver = Driver()
driver.setCruisingSpeed(cruise_speed)
driver.setSteeringAngle(steering_angle)    
 
camera = Camera('camera')
camera.enable(TIME_STEP)    

lidar = Lidar('Velodyne HDL-32E')
lidar.enable(TIME_STEP)
lidar.enablePointCloud()

gps = GPS('gps')
gps.enable(TIME_STEP)

gyro = Gyro('gyro')
gyro.enable(TIME_STEP)

def auto_driver():
  global traffic_light
  global cruise_speed
  global steering_angle
  if traffic_light=='None' or traffic_light=='Green':
    driver.setCruisingSpeed(cruise_speed)
  elif traffic_light=='Yellow':
    driver.setCruisingSpeed(cruise_speed/2)
  elif traffic_light=='Red':
    driver.setCruisingSpeed(0)
  driver.setSteeringAngle(steering_angle)

def callback_traffic_light( msg ):
  global traffic_light
  traffic_light = msg.data

def callback_cruise_speed( msg ):
  global cruise_speed
  cruise_speed = msg.data
 
def callback_steering_angle( msg ):
  global steering_angle
  steering_angle = msg.data

def main():

  print('RUNNING ROS CAR CONTROLLER NODE ...')
  rospy.init_node('ros_car_controller')
  rate = rospy.Rate(10)
  msg_image = Image()
  msg_image.height = camera.getHeight()
  msg_image.width = camera.getWidth()
  msg_image.is_bigendian = False
  msg_image.step = camera.getWidth() * 4
  msg_image.encoding = 'bgra8'
 
  msg_point_cloud = PointCloud2()
  msg_point_cloud.header.stamp = rospy.Time.now()
  msg_point_cloud.header.frame_id = 'lidar_link'  
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

  msg_gps = NavSatFix()
  msg_gps.header.stamp = rospy.Time.now()
  msg_gps.header.frame_id = 'gps_link'
  
  msg_gyro = Imu()
  msg_gyro.header.stamp = rospy.Time.now()
  msg_gyro.header.frame_id = 'gyro_link'
  pub_camera_data  = rospy.Publisher('/camera/rgb/raw', Image, queue_size=10)
  pub_point_cloud  = rospy.Publisher('/point_cloud'   , PointCloud2, queue_size=10)
  pub_nav_gps   = rospy.Publisher('/gps', NavSatFix, queue_size=10)
  pub_imu_gyro     = rospy.Publisher('/gyro', Imu, queue_size=10)

  rospy.Subscriber('/traffic_light'  , String, callback_traffic_light      )
  rospy.Subscriber('/goal_cruise_speed', Float64, callback_cruise_speed    )
  rospy.Subscriber('/goal_steering_angle', Float64, callback_steering_angle)

  while driver.step() != -1 and not rospy.is_shutdown():
    auto_driver()                                                

    msg_image.data = camera.getImage()                                          
    msg_point_cloud.data = lidar.getPointCloud(data_type='buffer')             
    msg_gyro.angular_velocity.x = gyro.getValues()[0]                          
    msg_gyro.angular_velocity.y = gyro.getValues()[1]                         
    msg_gyro.angular_velocity.z = gyro.getValues()[2]                         
    msg_gps.latitude = gps.getValues()[0]                                 
    msg_gps.longitude = gps.getValues()[1]                        
    msg_gps.altitude = gps.getValues()[2]                      
    
    
    pub_camera_data.publish(msg_image)                                    
    pub_point_cloud.publish(msg_point_cloud)                              
    pub_imu_gyro.publish(msg_gyro)                                     
    pub_nav_gps.publish(msg_gps)                                           
    
    rate.sleep()
    pass
 

if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass
