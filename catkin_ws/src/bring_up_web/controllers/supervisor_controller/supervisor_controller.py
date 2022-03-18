#!/usr/bin/env python3

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

# Z VELOCITY 
vel = [0.0, 0.0, 3.0, 0.0, 0.0, 0.0]
vel_1 = [5.0, 0.0, 0.0, 0.0, 0.0, 0.0]

def main():
    print('Starting Controller Supervisor...')

    i = 0
    j = 0
    while robot.step(TIME_STEP) != -1:
        if i == 0:                                                                  # SET INITIAL VELOCITY
            car_1.setVelocity(vel)
        elif i == 350:                                                              # RETURN TO INITIAL POSITION
            tf_1.setSFVec3f(sp_1)
            i = 0

        if j == 0:
            # car_2.setVelocity(vel_1)
            pass
        elif j == 150:
            # tf_2.setSFVec3f(sp_2)
            j = 0
    
        i+=1
        j+=1

if __name__ == "__main__":
    try:
        main()
    except:
        pass

