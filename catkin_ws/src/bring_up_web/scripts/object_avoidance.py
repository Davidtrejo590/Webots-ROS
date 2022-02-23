#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseArray


# GLOBAL VARIABLES
dist_tol = 10.0                                                     # TOLERANCE DISTANCE
min_dist_tol = -3.0                                                  # MIN TOLERANCE DISTANCE
vel_tol = 0.0                                                       # TOLERANCE VELOCITY                                                           

right_obstacle = False
left_obstacle = False
vertical_obstacle = False
vert_dist = 0.0

def callback_centroid_pose(msg):

    for obstacle in msg.poses:
        # obstacle - 1 OBSTACLE

        if (obstacle.position.x < 0.25 and obstacle.position.x > -0.25) and (obstacle.position.z < 0.0):                # OBJECT IN FRONT
            if (obstacle.position.z > min_dist_tol and obstacle.orientation.z < vel_tol):                               # OBJECT IS VERY CLOSE
                print('DISMINUIR VELOCIDAD Y MANTENER CARRIL')
                print('STOP')
            elif obstacle.position.z < dist_tol and obstacle.orientation.z < vel_tol:
                print('INICIAR ACCIÓN DE REBASE')
        else:                                                                                                           # THERE AREN'T OBJECTS IN FRONT
            print('VELOCIDAD CONSTANTE Y MANTENER CARRIL')
                        


def main():

    # INIT NODE
    print('Object avoidance node...')
    rospy.init_node('object_avoidance')
    rate = rospy.Rate(10)

    # SUBSCRIBERS
    rospy.Subscriber('/filter_pose', PoseArray, callback_centroid_pose)
    rospy.spin()



if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException




""" 
    MAQUINA DE ESTADOS PARA REBASE POR MEDIO DE ESTIMACIONES

    OBSERVAR EL CARRIL ACTUAL (EN EL QUE ESTOY )
    SI HAY OBJETO - ( OBJETO AL FRENTE EN MI CARRIL )
	    VERIFICAR DISTANCIA FRONTAL Y VELOCIDAD (EJE - Z)
	    SI distancia < tolerancia_minima && velocidad < min_vel
		    DISMINUIR LA VELOCIDAD HASTA 0, EL OBJETO ESTÁ MUY CERCA
	    SI distancia < tolerancia && velocidad < min_vel
            INICIAR ACCIÓN DE REBASE
	        OBSERVAR PARTE POSITIVA HORIZONTAL (EJE - X, DERECHA)
	        SI NO HAY OBJETO
		        OBSERVAR PARTE POSITIVA Y NEGATIVA VERTICAL (EJE - Z) 
                SI HAY OBJETO
                    SI (distancia_vertical > 10.0 && velocidad_vertical < vel_tol) || (distancia_vertical < -10.0 && velocidad_vertical < vel_tol)
                        GIRAR VOLANTE A LA DERECHA POR 't' SEGUNDOS - PUEDO PASAR AL CARRIL DERECHO (NO HAY OBSTÁCULO)
                    SI NO
                        DISMINUIR VELOCIDAD Y MANTENER EN CARRIL - HAY OBTÁCULO EN EL CARRIL DERECHO
                SI NO HAY OBJETO
                    GIRAR VOLANTE A LA DERECHA POR 't' SEGUNDOS - PUEDO PASAR AL CARRIL DERECHO (NO HAY OBSTÁCULO)
            SI HAY OBJETO
                OBSERVAR PARTE NEGATIVA HORIZONTAL (EJE - X, IZQUIERDA)
                SI NO HAY OBJETO
                    OBSERVAR PARTE POSITIVA Y NEGATIVA VERTICAL (EJE - Z)
                    SI HAY OBJETO
                        SI ( distancia_vertical > 10.0 && velocidad_vertical < vel_tol ) || ( distancia_vertical < -10.0 && velocidad_vertical < vel_tol )
                            GIRAR A LA IZQUIERDA POR 't' SEGUNDOS - PUEDO PASAR AL CARRIL IZQUIERDO (NO HAY OBSTÁCULO)
                        SI NO
                            DISMINUIR VELOCIDAD Y MANTENER CARRIL - HAY OBSTÁCULO EN EL CARRIL IZQUIERDO
                    SI NO HAY OBJETO
                        GIRAR A LA IZQUIERDA POT 't' SEGUNDOS - PUEDO PASAR AL CARRIL IZQUIERDO (NO HAY OBSTÁCULO)
                SI HAY OBJETO
                    DISMINUIR VELOCIDAD Y MANTENER CARRIL - HAY OBSTÁCULO EN EL CARRIL IZQUIERDO
    SI NO HAY OBJETO
	    MANTENER VELOCIDAD Y CARRIL

"""