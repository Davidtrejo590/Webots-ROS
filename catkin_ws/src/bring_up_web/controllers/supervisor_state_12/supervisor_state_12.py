#!/usr/bin/env python3

""" 
    CONTROLLER TO MOVE CARS IN VERTICAL LINE
    WITH CONSTANT SPEED TO TEST PASS & CRUISE BEHAVIORS
"""
import numpy
from controller import Supervisor

# TIME_STEP
TIME_STEP = 50

# SUPERVISOR
robot = Supervisor()

# NODES
car_N   = robot.getFromDef('CAR-N')
car_W   = robot.getFromDef('CAR-W')
car_NW  = robot.getFromDef('CAR-NW')
car_SW  = robot.getFromDef('CAR-SW')

# GET INITIAL POSITION FOR EACH CAR
sp_N    = car_N.getPosition()
sp_W    = car_W.getPosition()
sp_NW   = car_NW.getPosition()
sp_SW   = car_SW.getPosition()

# GET THE TRANSLATION FIELD FOR EACH CAR
tf_N    = car_N.getField('translation')
tf_W    = car_W.getField('translation')
tf_NW   = car_NW.getField('translation')
tf_SW   = car_SW.getField('translation')

# FOR STATE 00:
initial_N   = [0,0,1000]
initial_W   = [0,0,1000]
initial_NW  = [0,0,1000]
initial_SW  = [0,0,1000]

tf_N.setSFVec3f(sp_N)
tf_W.setSFVec3f(initial_W)
tf_NW.setSFVec3f(sp_NW)
tf_SW.setSFVec3f(initial_SW)

v_N     = numpy.random.uniform(3.0, 4.0)
v_W     = numpy.random.uniform(0.0, 0.0)
v_NW    = numpy.random.uniform(3.0, 4.0)
v_SW    = numpy.random.uniform(0.0, 0.0)

vel_N  = [0.0, 0.0,  v_N, 0.0, 0.0, 0.0]            # Z VELOCITY FOR CAR N 
vel_W  = [0.0, 0.0,  v_W, 0.0, 0.0, 0.0]            # X VELOCITY DOR CAR W
vel_NW = [0.0, 0.0,  v_NW, 0.0, 0.0, 0.0]           # Z VELOCITY FOR CAR NW 
vel_SW = [0.0, 0.0,  v_SW, 0.0, 0.0, 0.0]           # X VELOCITY DOR CAR SW

print("CAR N VELOCITY: " + str(v_N))
print("CAR NW VELOCITY: " + str(v_NW))

# MAIN FUNCTION
def main():
    print('Starting Controller Supervisor for CASE experiment state 12...')

    i = 0
    while robot.step(TIME_STEP) != -1:
        if i == 0:                              # SET INITIAL VELOCITY
            car_N.setVelocity(vel_N)
            car_W.setVelocity(vel_W)
            car_NW.setVelocity(vel_NW)
            car_SW.setVelocity(vel_SW)
        elif i == 350:                          # RETURN TO INITIAL POSITION CAR 1
            tf_N.setSFVec3f(sp_N)
            tf_W.setSFVec3f(sp_W)
            tf_NW.setSFVec3f(sp_NW)
            tf_SW.setSFVec3f(sp_SW)
            i = 0
        i+=1


if __name__ == "__main__":
    try:
        main()
    except:
        pass

