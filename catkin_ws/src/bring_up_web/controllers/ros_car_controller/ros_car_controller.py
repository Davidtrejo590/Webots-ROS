#!/usr/bin/env python3

""" ros_car_controller """

# LIBRARIES
import math
from vehicle import Driver
from controller import Camera, Keyboard, Lidar, Gyro, GPS
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Image, PointCloud2, PointField, NavSatFix, NavSatStatus, Imu

# CONSTANTS
TIME_STEP = 50
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
# lidar = Lidar('Sick LMS 291')
lidar = Lidar('lidar')
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
  global left_angle
  global right_angle
  key = keyboard.getKey()
  # CALCULATE LEFT STEERING ANGLE
  if(key == keyboard.LEFT):
    left_angle = driver.getSteeringAngle() - (0.0174533*5)
    driver.setSteeringAngle(left_angle)
    print('Left Angle', driver.getSteeringAngle())
  # CALCULATE RIGHT STEERING ANGLE
  elif (key == keyboard.RIGHT):
    right_angle = driver.getSteeringAngle() + (0.0174533*5)
    driver.setSteeringAngle(right_angle)
    print('Right Angle', driver.getSteeringAngle())
  elif (key == keyboard.UP):
    driver.setCruisingSpeed(10.0)
  elif (key == keyboard.DOWN):
    driver.setCruisingSpeed(0.0)

# HELP
def help():
  print('TO MOVE CAR SELECT THE 3D WINDOW AND USE THE KEYS TO: \n')
  print('[LEFT]/[RIGHT] - STEERING ANGLE')
  print('[UP]/[DOWN] - SET CONSTANT SPEED / SLOW DOWN')

# CRUISE SPEED CALLBACK
def callback_cruise_speed( msg ):
  driver.setCruisingSpeed(msg.data)

# STEERING ANGLE CALLBACK
def callback_steering_angle(msg):
  driver.setSteeringAngle(msg.data)

# MAIN FUNCTION
def main():

  # INIT ROS
  print('RUNNING ROS CAR CONTROLLER NODE ...')
  rospy.init_node('ros_car_controller')
  rate = rospy.Rate(20)

  # PRINT HELP FOR USER
  help()                        

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

  # GPS MESSAGE
  msg_gps = NavSatFix()
  msg_gps.header.stamp = rospy.Time.now()
  msg_gps.header.frame_id = 'gps_link'
  # msg_gps.position_covariance = NavSatFix.COVARIANCE_TYPE_KNOWN
  # msg_gps.status.service = NavSatStatus.SERVICE_GPS
  

  # GYRO MESSAGE
  msg_gyro = Imu()
  msg_gyro.header.stamp = rospy.Time.now()
  msg_gyro.header.frame_id = 'gyro_link'
  

  # PUBLISHERS
  pub_camera_data  = rospy.Publisher('/camera/rgb/raw', Image, queue_size=10)
  pub_point_cloud  = rospy.Publisher('/point_cloud'   , PointCloud2, queue_size=10)
  pub_nav_gps   = rospy.Publisher('/gps', NavSatFix, queue_size=10)
  pub_imu_gyro     = rospy.Publisher('/gyro', Imu, queue_size=10)

  # SUBSCRIBERS
  rospy.Subscriber('/goal_cruise_speed'  , Float64, callback_cruise_speed  )
  rospy.Subscriber('/goal_steering_angle', Float64, callback_steering_angle)

  # MAIN LOOP
  while driver.step() != -1 and not rospy.is_shutdown():
    check_keyboard()                                                                    # CHECK KEYBOARD

    msg_image.data = camera.getImage()                                                  # GET IMAGE DATA FROM CAMERA
    msg_point_cloud.data = lidar.getPointCloud(data_type='buffer')                      # GET POINT CLOUD FROM LIDAR
    msg_gyro.angular_velocity.x = gyro.getValues()[0]                                   # GET X COMPONENT FROM GYRO
    msg_gyro.angular_velocity.y = gyro.getValues()[1]                                   # GET Y COMPONENT FROM GYRO
    msg_gyro.angular_velocity.z = gyro.getValues()[2]                                   # GET Z COMPONENT FROM GYRO
    msg_gps.latitude = gps.getValues()[0]                                               # GET X COMPONENT FROM GPS  
    msg_gps.longitude = gps.getValues()[1]                                              # GET Y COMPONENT FROM GPS  
    msg_gps.altitude = gps.getValues()[2]                                               # GET Z COMPONENT FROM GPS 
    
    
    pub_camera_data.publish(msg_image)                                                  # PUBLISHING IMAGE MESSAGE
    pub_point_cloud.publish(msg_point_cloud)                                            # PUBLISHING POINTCLOUD2 MESSAGE
    pub_imu_gyro.publish(msg_gyro)                                                      # PUBLISHING IMU MESSAGE
    pub_nav_gps.publish(msg_gps)                                                        # PUBLISHING NAVSATFIX MESSAGE
    
    rate.sleep()
    pass
 

if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass
