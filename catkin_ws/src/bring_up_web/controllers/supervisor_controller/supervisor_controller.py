#!/usr/bin/env python3

from controller import Supervisor

# TIME_STEP
TIME_STEP = 64

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

# Z VELOCITY 
vel = [0.0, 0.0, 5.0, 0.0, 0.0, 0.0]
vel_1 = [0.0, 0.0, -5.0, 0.0, 0.0, 0.0]

def main():
    print('Starting Controller Supervisor...')

    i = 0
    while robot.step(TIME_STEP) != -1:
        if i == 0:                                                                  # SET INITIAL VELOCITY
            # car_1.setVelocity(vel)
            # car_2.setVelocity(vel)
            # car_3.setVelocity(vel_1)
            # car_4.setVelocity(vel_1)
            print('')
        elif i == 190:
            # tf_2.setSFVec3f(sp_2)
            # tf_4.setSFVec3f(sp_4)
            print('')
        elif i == 380:                                                              # RETURN TO INITIAL POSITION
            # tf_1.setSFVec3f(sp_1)
            # tf_2.setSFVec3f(sp_2)
            # tf_3.setSFVec3f(sp_3)
            # tf_4.setSFVec3f(sp_4)
            i = 0
    
        i+=1

if __name__ == "__main__":
    try:
        main()
    except:
        pass

