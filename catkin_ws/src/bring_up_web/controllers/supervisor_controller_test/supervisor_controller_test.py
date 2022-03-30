#!/usr/bin/env python3

""" 
    CONTROLLER TO MOVE CARS IN VERTICAL LINE
    WITH CONSTANT SPEED TO TEST PASS & CRUISE BEHAVIORS
"""

from controller import Supervisor

# TIME_STEP
TIME_STEP = 50

# SUPERVISOR
robot = Supervisor()

# NODES
car_1 = robot.getFromDef('CAR-1')
car_2 = robot.getFromDef('CAR-2')

# GET INITIAL POSITION FOR EACH CAR
sp_1 = car_1.getPosition()
sp_2 = car_2.getPosition()

# GET THE TRANSLATION FIELD FOR EACH CAR
tf_1 = car_1.getField('translation')
tf_2 = car_2.getField('translation')

vel     = [0.0, 0.0, 5.0, 0.0, 0.0, 0.0]            # Z VELOCITY FOR CAR 1 


# MAIN FUNCTION
def main():
    print('Starting Supervisor Test Controller...')

    i = 0
    j = 0
    while robot.step(TIME_STEP) != -1:
        if i == 0:                              # SET INITIAL VELOCITY
            car_1.setVelocity(vel)
            car_2.setVelocity(vel)
        elif i == 500:                          # RETURN TO INITIAL POSITION CAR 1
            tf_1.setSFVec3f(sp_1)
            tf_2.setSFVec3f(sp_2)
            i = 0
    
        i+=1


if __name__ == "__main__":
    try:
        main()
    except:
        pass

