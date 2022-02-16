from controller import Supervisor


TIME_STEP = 32

# SUPERVISOR
robot = Supervisor()

# NODES
car_1 = robot.getFromDef('CAR-1')
car_2 = robot.getFromDef('CAR-2')
car_3 = robot.getFromDef('CAR-3')
car_4 = robot.getFromDef('CAR-4')

sp_1 = car_1.getPosition()
sp_2 = car_2.getPosition()
sp_3 = car_3.getPosition()
sp_4 = car_4.getPosition()

tf_1 = car_1.getField('translation')
tf_2 = car_2.getField('translation')
tf_3 = car_3.getField('translation')
tf_4 = car_4.getField('translation')

vel = [0.0, 0.0, 10.0, 0.0, 0.0, 0.0]
vel_1 = [0.0, 0.0, -10.0, 0.0, 0.0, 0.0]

i = 0

while robot.step(TIME_STEP) != -1:
    if i == 0:
        car_1.setVelocity(vel)
        car_2.setVelocity(vel_1)
        car_3.setVelocity(vel_1)
        car_4.setVelocity(vel)
    elif i == 187:
        tf_1.setSFVec3f(sp_1)
        tf_2.setSFVec3f(sp_2)
        tf_3.setSFVec3f(sp_3)
        tf_4.setSFVec3f(sp_4)
        i = 0
    
    i+=1

