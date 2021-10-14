"""epuck_avoid_collision controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, DistanceSensor, Motor

# DEFINE TIME STEP
TIME_STEP = 64

# ROBOT INSTANCE
robot = Robot()

# INITIALIZE DEVICES
ps = []
psNames = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']

# GET AND ENABLE THE DISTANCE SENSORS 
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)
    
# INITIALIZE THE MOTORS
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)




# Main loop:
while robot.step(TIME_STEP) != -1:
    
    # READ SENSORS OUTPUTS
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())
       
    # PROCCESS BEHAVIOR 
    right_obstacle = psValues[0] > 80.0 or psValues[1] > 80.0 or psValues[2] > 80.0
    left_obstacle = psValues[5] > 80.0 or psValues[6] > 80.0 or psValues[7] > 80.0
    
    # WRITE ACTUATORS OUTPUT
    MAX_SPEED = 6.28
    
    # INITIALIZE MOTOR SPEEDS
    left_speed = 0.5 * MAX_SPEED
    right_speed = 0.5 * MAX_SPEED
    
    # MODIFY SPEEDS ACCORDING TO OBSTACLES
    if left_obstacle:
         # TURN RIGHT
        left_speed = 0.5 * MAX_SPEED
        right_speed = -0.5 * MAX_SPEED
    elif right_obstacle:
         # TUNR LEFT
        left_speed = -0.5 * MAX_SPEED
        right_speed = 0.5 * MAX_SPEED
     
    leftMotor.setVelocity(left_speed)
    rightMotor.setVelocity(right_speed)
         
    
    pass

# Enter here exit cleanup code.

