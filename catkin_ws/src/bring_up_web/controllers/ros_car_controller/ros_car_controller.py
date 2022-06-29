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
from std_msgs.msg import Float64
from sensor_msgs.msg import Image, PointCloud2, PointField


# INIT DRIVER
driver = Driver()
driver.setCruisingSpeed(0.0)                                  # SPEED CONTROL km/h - INITIAL SPEED
driver.setSteeringAngle(0.0)                                  # STEERING ANGLE - INITIAL ANGLE
timestep = int(driver.getBasicTimeStep())                     # BASIS TIME STEP FROM CRURRENT WORLD

# INIT CAMERA
camera = Camera('camera')                                     # GET CAMERA FROM DEVICES
camera.enable(timestep)                                       # ENABLE CAMERA

# INIT LIDAR
lidar = Lidar('lidar')                                        # GET LIDAR FROM DEVICES
lidar.enable(timestep)                                        # ENABLE LIDAR
lidar.enablePointCloud()

# INIT KEYBOARD
keyboard = Keyboard()                                         # ENABLE KEYBORAR
keyboard.enable(timestep)                                     # ENABLE KEYBORAD

# CHECK KEYBORARD TO SET STEERING ANGLE & SPEED FROM KEYBORAD
def check_keyboard():

  left_steering   = 0.0 
  right_steering  = 0.0

  # CHECK KEYBOARD
  key = keyboard.getKey()

  if key == keyboard.LEFT:                                        # COMPUTE & SET LEFT STEERING ANGLE
    left_steering = driver.getSteeringAngle() - ( 0.0174533 * 5 )
    driver.setSteeringAngle(left_steering)

  elif key == keyboard.RIGHT:                                     # COMPUTE & SET RIGHT STEERING ANGLE
    right_steering = driver.getSteeringAngle() + (0.0174533*5)
    driver.setSteeringAngle(right_steering)

  elif (key == keyboard.UP):                                      # SET CRUISE SPEED
    driver.setCruisingSpeed(10.0)

  elif (key == keyboard.DOWN):                                    # SET CRUISE SPEED
    driver.setCruisingSpeed(0.0)


# FUNCTION TO GIVE HELP
def help():
  print('TO MOVE CAR SELECT THE 3D WINDOW AND USE THE KEYS TO: \n')
  print('[LEFT]/[RIGHT] - STEERING ANGLE')
  print('[UP]/[DOWN] - SET CONSTANT SPEED / SLOW DOWN')

# CRUISE SPEED CALLBACK
def callback_speed( msg ):
  driver.setCruisingSpeed(msg.data)

# STEERING ANGLE CALLBACK
def callback_steering( msg ):
  driver.setSteeringAngle(msg.data)

# MAIN FUNCTION
def main():

  # INIT ROS
  print('RUNNING ROS CAR CONTROLLER NODE ...')
  rospy.init_node('ros_car_controller')
  rate = rospy.Rate(10)

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
  

  # PUBLISHERS
  pub_steering_angle  = rospy.Publisher('/current_steering', Float64, queue_size=10)
  pub_camera_data     = rospy.Publisher('/camera/rgb/raw', Image, queue_size=10)
  pub_point_cloud     = rospy.Publisher('/point_cloud'   , PointCloud2, queue_size=10)
  
  # SUBSCRIBERS
  rospy.Subscriber('/goal_speed', Float64, callback_speed)
  rospy.Subscriber('/goal_steering', Float64, callback_steering)

  # MAIN LOOP
  while driver.step() != -1 and not rospy.is_shutdown():
    
    check_keyboard()                                                      # CHECK KEYBOARD

    msg_image.data = camera.getImage()                                    # GET IMAGE DATA FROM CAMERA
    msg_point_cloud.data = lidar.getPointCloud(data_type='buffer')        # GET POINT CLOUD FROM LIDAR
    msg_point_cloud.header.stamp = rospy.Time.now()                       # REFRESH STAMP FOR POINT CLOUD
    
    pub_steering_angle.publish(driver.getSteeringAngle())                 # PUBLISH CURRENT STEERING ANGLE
    pub_point_cloud.publish(msg_point_cloud)                              # PUBLISHING POINTCLOUD2 MESSAGE
    pub_camera_data.publish(msg_image)                                    # PUBLISHING IMAGE MESSAGE
    
    rate.sleep()


if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass
