#!/usr/bin/env python3

""" 
    MAIN CONTROLLER TO 
    GET INFORMATION OF THE SENSORS (LIDAR, CAMERA)
    SET CRUISE SPEED & STEERING ANGLE

"""

# LIBRARIES
import rospy
from vehicle import Driver
from controller import Camera, Keyboard, Lidar
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import Image, PointCloud2, PointField

# CONSTANTS
TIME_STEP = 50

# GLOBAL VARIABLES 
start = False

# INIT DRIVER
driver = Driver()
driver.setCruisingSpeed(0.0)                                  # SPEED CONTROL km/h - INITIAL SPEED
driver.setSteeringAngle(0.0)                                  # STEERING ANGLE - INITIAL ANGLE

# INIT CAMERA
camera = Camera('camera')                                     # GET CAMERA FROM DEVICES
camera.enable(TIME_STEP)    

# INIT LIDAR
lidar = Lidar('lidar')                                        # GET LIDAR FROM DEVICES
lidar.enable(TIME_STEP)
lidar.enablePointCloud()

# INIT KEYBOARD
keyboard = Keyboard()
keyboard.enable(TIME_STEP)

# CHECK KEYBORARD TO SET STEERING ANGLE & SPEED FROM KEYBORAD
def check_keyboard():

  global start

  left_angle = 0.0 
  right_angle = 0.0

  # CHECK KEYBOARD
  key = keyboard.getKey()

  if key == keyboard.LEFT:                                        # COMPUTE & SET LEFT STEERING ANGLE
    left_angle = driver.getSteeringAngle() - ( 0.0174533 * 5 )
    driver.setSteeringAngle(left_angle)

  elif key == keyboard.RIGHT:                                     # COMPUTE & SET RIGHT STEERING ANGLE
    right_angle = driver.getSteeringAngle() + (0.0174533*5)
    driver.setSteeringAngle(right_angle)

  elif (key == keyboard.UP):                                      # SET CRUISE SPEED
    driver.setCruisingSpeed(10.0)
    print('UP')

  elif (key == keyboard.DOWN):                                    # SET CRUISE SPEED
    driver.setCruisingSpeed(0.0)
  
  elif (key == ord('S')):
    start = True

# FUNCTION TO GIVE HELP
def help():
  print('TO MOVE CAR SELECT THE 3D WINDOW AND USE THE KEYS TO: \n')
  print('[LEFT]/[RIGHT] - STEERING ANGLE')
  print('[UP]/[DOWN] - SET CONSTANT SPEED / SLOW DOWN')

# CRUISE SPEED CALLBACK
def callback_cruise_speed( msg ):
  driver.setCruisingSpeed(msg.data)

# STEERING ANGLE CALLBACK
def callback_steering_angle( msg ):
  driver.setSteeringAngle(msg.data)

# MAIN FUNCTION
def main():

  global start

  # INIT ROS
  print('RUNNING ROS CAR CONTROLLER NODE ...')
  rospy.init_node('ros_car_controller')
  rate = rospy.Rate(20)

  # PRINT HELP FOR USER
  help()                        

  # BOOL MESSAGE
  msg_bool = Bool()

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
  

  # PUBLISHERS
  pub_camera_data  = rospy.Publisher('/camera/rgb/raw', Image, queue_size=10)
  pub_point_cloud  = rospy.Publisher('/point_cloud'   , PointCloud2, queue_size=10)
  pub_steering_angle = rospy.Publisher('/current_steering', Float64, queue_size=10)
  pub_enable_start = rospy.Publisher('/enable_start', Bool, queue_size=10)

  # SUBSCRIBERS
  rospy.Subscriber('/goal_cruise_speed'  , Float64, callback_cruise_speed  )
  rospy.Subscriber('/goal_steering_angle', Float64, callback_steering_angle)

  # MAIN LOOP
  while driver.step() != -1 and not rospy.is_shutdown():
    
    check_keyboard()                                                      # CHECK KEYBOARD

    pub_steering_angle.publish(driver.getSteeringAngle()) 
    msg_image.data = camera.getImage()                                    # GET IMAGE DATA FROM CAMERA
    msg_point_cloud.data = lidar.getPointCloud(data_type='buffer')        # GET POINT CLOUD FROM LIDAR
    msg_point_cloud.header.stamp = rospy.Time.now()                       # REFRESH STAMP FOR POINT CLOUD
    msg_bool.data = start                                                 # GET ENABLE START FROM KEYBOARD
    
    pub_enable_start.publish(msg_bool)                                    # PUBLISHING START FLAG
    pub_camera_data.publish(msg_image)                                    # PUBLISHING IMAGE MESSAGE
    pub_point_cloud.publish(msg_point_cloud)                              # PUBLISHING POINTCLOUD2 MESSAGE
    
    rate.sleep()

 

if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass
