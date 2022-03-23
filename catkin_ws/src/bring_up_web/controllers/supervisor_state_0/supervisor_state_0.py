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
car_1 = robot.getFromDef('CAR-1')
car_2 = robot.getFromDef('CAR-2')
car_3 = robot.getFromDef('CAR-3')
car_4 = robot.getFromDef('CAR-4')

# GET INITIAL POSITION FOR EACH CAR
sp_1 = car_1.getPosition()
sp_2 = car_2.getPosition()
sp_3 = car_3.getPosition()
sp_4 = car_4.getPosition()

# GET THE TRANSLATION FIELD FOR EACH CAR
tf_1 = car_1.getField('translation')
tf_2 = car_2.getField('translation')
tf_3 = car_3.getField('translation')
tf_4 = car_4.getField('translation')

#### FOR STATE 0:
initial_1 = [0,0,1000]
initial_2 = [0,0,1000]
initial_3 = [0,0,1000]
initial_4 = [0,0,1000]
tf_1.setSFVec3f(initial_1)
tf_2.setSFVec3f(initial_2)
tf_3.setSFVec3f(initial_3)
tf_4.setSFVec3f(initial_4)

v1 = numpy.random.uniform(0.0, 0)
v2 = numpy.random.uniform(0.0, 0)
v3 = numpy.random.uniform(0.0, 0)
v4 = numpy.random.uniform(0.0, 0)

vel_1  = [0.0, 0.0,  v1, 0.0, 0.0, 0.0]            # Z VELOCITY FOR CAR 1 
vel_2  = [0.0, 0.0,  v2, 0.0, 0.0, 0.0]            # X VELOCITY DOR CAR 2
vel_3  = [0.0, 0.0,  v3, 0.0, 0.0, 0.0]            # Z VELOCITY FOR CAR 3 
vel_4  = [0.0, 0.0,  v3, 0.0, 0.0, 0.0]            # X VELOCITY DOR CAR 4


# MAIN FUNCTION
def main():
    print('Starting Controller Supervisor for CASE experiment state 0...')

    i = 0
    j = 0
    while robot.step(TIME_STEP) != -1:
        if i == 0:                              # SET INITIAL VELOCITY
            car_1.setVelocity(vel_1)
            car_2.setVelocity(vel_2)
            car_3.setVelocity(vel_3)
            car_4.setVelocity(vel_4)
        elif i == 500:                          # RETURN TO INITIAL POSITION CAR 1
            tf_1.setSFVec3f(initial_1)
            tf_2.setSFVec3f(initial_2)
            tf_3.setSFVec3f(initial_3)
            tf_4.setSFVec3f(initial_4)
            i = 0
        i+=1


if __name__ == "__main__":
    try:
        main()
    except:
        pass

