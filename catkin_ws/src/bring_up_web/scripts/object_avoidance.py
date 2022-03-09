#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseArray

# STATES
SM_INIT = 'INIT'
SM_C = 'CRUISE'
SM_KD = 'KEEP DISTANCE'
SM_CL = 'CHANGE LANE'

# CAR OBSTACLES - EVERYTHING IS FREE
fn  = 1
fnw = 1
fsw = 1
fw  = 1

def callback_centroid_pose(msg):

    global fn, fnw, fsw, fw
    cars = []
    for car in msg.poses:
        cars.append([car.position.x, car.position.z])

    
    # BUSCAR COCHE AL NORTE
    for c in cars:
        if (c[0] > -0.5 and c[0] < 0.5) and (c[1] > -15.0 and c[1] < -10.0):
        # COCHE AL FRENTE
            fn = 0
            break

    if fn:                              # SI NORTE LIBRE
        print('CRUCERO')
    else:
        # BUSCAR COCHE AL OESTE
        for c in cars:
            if (c[0] > -4.0 and c[0] < -3.0) and ( c[1] > -0.5 and c[1] < 0.5):
                # COCHE AL OESTE
                fw = 0
                break
        if fw:                          # SI OESTE LIBRE
            # BUSCAR COCHE AL NOROESTE
            for c in cars:
                if (c[0] > -4.0 and c[0] < -3.0) and ( c[1] > -15.0 and c[1] < -10.0):
                    # COCHE AL NOROESTE
                    fnw = 0
                    break
            if fnw:                     # SI NOROESTE LIBRE
                # BUSCAR COCHE AL SUROESTE
                for c in cars:
                    if (c[0] > -4.0 and c[0] < -3.0) and (c[1] < 15.0 and c[1] > 10.0):
                        # COCHE AL SUROESTE
                        fsw = 0
                        break
                if fsw:                 # SI SUROESTE LIBRE
                    # CAMBIAR DE CARRIL
                    print('CAMBIAR DE CARRIL')
                else:
                    # MANTENER DISTANCIA
                    print('MANTENER DISTANCIA')
            else:
                # MANTENER DISTANCIA
                print('MANTENER DISTANCIA')
        else:
            # MANTENER DISTANCIA
            print('MANTENER DISTANCIA')


    # COMPORTAMIENTO
    print('[FW, FSW, FNW, FN]', [fw, fsw, fnw, fn])
     


def main():

    # INIT NODE
    print('Object avoidance node...')
    rospy.init_node('object_avoidance')
    rate = rospy.Rate(10)

    # SUBSCRIBERS
    rospy.Subscriber('/filter_pose', PoseArray, callback_centroid_pose)
    
    global fn, fnw, fsw, fw

    state = SM_INIT
    # STATE MACHINE
    while not rospy.is_shutdown():
        if state == SM_INIT:
            if fn:
                state = SM_C
            else:
                state = SM_KD
        elif state == SM_C:
            state = SM_C



        rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException
