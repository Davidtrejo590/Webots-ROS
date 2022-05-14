# Navegación autónoma para un vehículo sin conductor usando el simulador Webots

Desarrollo de  algoritmos para navegación de autos autónomos usando el simulador Webots y la plataforma ROS.

## REQUERIMIENTOS

* Ubuntu 18.04
* ROS-Melodic
* Webots R2021b

## INSTALACIÓN 

* $ cd
* $ git clone https://github.com/Davidtrejo590/Webots-ROS.git
* $ cd Webots-ROS
* $ cd catkin_ws
* $ catkin_make

## PARA PROBAR 

* $ export WEBOTS_HOME=/usr/local/webots
* $ cd
* $ source Webots-ROS/catkin_ws/devel/setup.bash
* $ roslaunch bring_up_web test.launch

* Si todo se instaló y compiló correctamente se debe de visualizar un Webots como el siguiente:
![webots_test](https://raw.githubusercontent.com/Davidtrejo590/Webots-ROS/master/Media/webots.png)

* Además de un RViz como el siguiente:
![rviz_test](https://raw.githubusercontent.com/Davidtrejo590/Webots-ROS/master/Media/rviz.png)

## CONTACTO
Luis David Torres Trejo<br>
luisdavidtorrestrejo@gmail.com<br>




