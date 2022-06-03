#!/usr/bin/env python3

""" 
    CONTROLLER TO MOVE CARS IN VERTICAL LINE
    WITH CONSTANT SPEED TO TEST PASS & CRUISE BEHAVIORS
"""

from controller import Supervisor

# TIME_STEP
TIME_STEP = 100

# SUPERVISOR
robot = Supervisor()

# NODES
car_1 = robot.getFromDef('CAR-1')

# GET INITIAL POSITION FOR EACH CAR
sp_1 = car_1.getPosition()

# GET THE TRANSLATION FIELD FOR EACH CAR
tf_1 = car_1.getField('translation')

# Z VELOCITY FOR CAR 1 
vel = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]    


# MAIN FUNCTION
def main():
    print('Starting Controller Supervisor...')

    i = 0
    while robot.step(TIME_STEP) != -1:
        if i == 0:                              # SET INITIAL VELOCITY
            car_1.setVelocity(vel)
        elif i == 350:                          # RETURN TO INITIAL POSITION CAR 1
            tf_1.setSFVec3f(sp_1)
            i = 0
        
        i+=1


if __name__ == "__main__":
    try:
        main()
    except:
        pass

