#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseArray


# GLOBAL VARIABLES
tol = 10.0
min_tol = 5.0
right_obstacle = False
left_obstacle = False
vertical_obstacle = False
vert_dist = 0.0

def callback_centroid_pose(msg):

    for centroid in msg.poses:
        if (centroid.position.x < 0.25 and centroid.position.x > -0.25) and (centroid.position.z < 0.0):                # OBJECT IN FRONT
            if centroid.position.z < min_tol:                                                                           # OBJECT IS VERY CLOSE
                print('DISMINUIR VELOCIDAD Y MANTENER CARRIL')
                print('STOP')
            elif centroid.position.z < tol:                                                                             # OBJECT IS CLOSE
                print('OBSERVAR PARTE HORIZONTAL POSITIVA')
                if not right_obstacle:                                                                                  # NO OBSTACLE IN RIGHT
                    print('OBSERVAR PARTE VERTICAL POSITIVA Y NEGATIVA')
                    if vertical_obstacle:                                                                               # CHECK OBJECT IN VERTICAL AXIS
                        if vert_dist > 10.0 or vert_dist < -10.0:                                                       # CHECK RANGE IN Z AXIS
                            print('GIRAR VOLANTE A LA DERECHA POR "1s" ')
                        else:
                            print('DISMINUIR VELOCIDAD Y MANTENER CARRIL')
                    else:
                        print('GIRAR VOLANTE A LA DERECHA POR "1s" ')

                print('OBSERVAR PARTE HORIZONTAL NEGATIVA')
                if not left_obstacle:                                                                                   # NO OBSTACLE IN LEFT
                    print('OBSREVAR PARTE VERTICAL POSITIVA Y NEGATIVA')
                    if vertical_obstacle:                                                                               # CHECK OBJECT IN VERTICAL AXIS
                        if vert_dist > 10.0 or vert_dist < -10.0:                                                       # CHECK RANGE IN Z AXIS
                            print('GIRAR A LA IZQUIERDA POR "1s" ')
                        else:
                            print('DISMINUIR VELOCIDAD Y MANTENER CARRIL')
                    else:
                        print('GIRAR A LA IZQUIERDA POR "1s" ')
                else:                                                                                                   # THERE ARE OBJECTS IN LEFT & RIGHT
                    print('DISMINUIR VELOCIDAD Y MANTENER CARRIL')
            else:                                                                                                       # THERE AREN'T OBJECTS IN FRONT
                print('VELOCIDAD CONSTANTE Y MANTENER CARRIL')
        else:
            print('VELOCIDAD CONSTANTE Y MANTENER CARRIL')
                        


def main():

    # INIT NODE
    print('Object avoidance node...')
    rospy.init_node('object_avoidance')
    rate = rospy.Rate(10)

    # SUBSCRIBERS
    rospy.Subscriber('/centroid_pose', PoseArray, callback_centroid_pose)
    rospy.spin()



if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException
