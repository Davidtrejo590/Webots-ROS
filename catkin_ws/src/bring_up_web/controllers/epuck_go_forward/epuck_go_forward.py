"""epuck_go_forward controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor

TIME_STEP = 64
MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

# GET THE MOTOR DEVICES 
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')

# SET THE TARGET POSITION OF THE MOTORS 
#leftMotor.setPosition(20.0)
#rightMotor.setPosition(20.0)

# GET A HANDLER TO THE MOTORS AND SET TARGET POSITION TO INFINITY (SPEED CONTROL)
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))


# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# SET UP THE MOTOR SPEEDS AT 10% OF THE MAX_SPEED
leftMotor.setVelocity(0.1 * MAX_SPEED)
rightMotor.setVelocity(0.1 * MAX_SPEED)


# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
