# Proyecto Roboracer con Librerìa F1Tenth

## Recorrido de circuito con Algoritmo de F1Tenth
El coche puede ser visualizado en rviz y al ejecutar el archivo obstaculosFTG, iniciarà el recorrido del circuito segùn el mapa designado en el archivo sim.yaml.
```bash
#F1tenth
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
ros2 run controllers obstaculosFTG
```
## Gazebo con F1Tenth
Se implementò tambièn la simulaciòn en Gazebo donde se cuenta con el vehìculo con su lidar y càmara de profundidad, por el momento no se puede trasladar el movimiento del robot hacia Gazebo porque hay problemas con los topicos al usar ros2_control.

Los siguientes comandos son para ejecutar la simulaciòn en Gazebo y trabajan con el xacro gazebo_racecar.xacro.
```bash
# F1tenth con gazebo
# Mundo vacío (default)
ros2 launch f1tenth_gym_ros gazebo_test.launch.py

# Con mundo personalizado
ros2 launch f1tenth_gym_ros gazebo_test.launch.py world:=./src/f1tenth_gym_ros/worlds/levine.world
```

Para instalar dependencias necesarias, seguir los primeros pasos de este repositorio de git:

https://github.com/widegonz/F1Tenth-Repository
