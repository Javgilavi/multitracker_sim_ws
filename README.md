# Multitracker Simulation Project

Este proyecto está desarrollado para **ROS2 Humble** con el uso de **GAZEBO Classic** en **Ubuntu 22.04**. El proyecto consta de tres paquetes dentro de `src`, que se utilizan para simular un entorno de seguimiento de entidades mediante la fusión sensorial de un LIDAR y datos ADS-B.

## Estructura del proyecto

- **drone_sim**: Contiene el entorno de simulación en Gazebo, junto con nodos complementarios para mover el dron y los cubos.
- **multitracker**: Nodo que actúa como multitracker de entidades, utilizando datos de sensores.
- **sim_msgs**: Paquete que contiene los mensajes personalizados utilizados en el proyecto, como `Adsb.msg`.

## Descripción general

Este proyecto simula un dron con un sensor **LIDAR** que realiza el seguimiento de tres entidades (cubos) mediante la fusión de datos sensoriales. Los cubos publican periódicamente sus estados a través de mensajes **ADS-B** en sus respectivos tópicos, lo que permite al dron realizar un seguimiento en tiempo real.

- El dron publica su estado en el tópico `/drone/state`.
- Los cubos publican su estado en el tópico `/cube/state`.
- Si se utiliza el LIDAR, el sensor publicará mensajes de tipo **PointCloud** en el tópico `/drone/laser/scan`.

El nodo **multitracker** utiliza un filtro de Kalman para predecir y actualizar el estado de las entidades, y los trackeos se visualizan en RViz2 a través del tópico `/cube/marker`.

## Requisitos

- **ROS2 Humble**
- **Gazebo Classic**
- **Ubuntu 22.04**

## Instrucciones de compilación

1. **Compilar el proyecto**:
   Primero, compila el paquete `sim_msgs`, seguido del resto del proyecto.

   ```bash
   colcon build --packages-select sim_msgs
   colcon build
   ```

2. **Fuente del entorno**:
   Tras compilar, ejecuta:

   ```bash
   source install/setup.bash
   ```

## Ejecución de la simulación en Gazebo

1. **Lanzar la simulación**:
   Para iniciar el entorno de simulación en Gazebo, utiliza el siguiente comando:

   ```bash
   ros2 launch drone_sim drone_world_launch.py
   ```

   En el archivo de lanzamiento <drone_sim/launch/drone_world_launch.py>, en la línea 16, puedes cambiar `model.sdf` por `model_lidar.sdf` para incluir o excluir el sensor LIDAR en el dron.

2. **Mover los cubos aleatoriamente**:
   Para mover los cubos de forma aleatoria en la simulación, ejecuta:

   ```bash
   ros2 run drone_sim cube_mover
   ```

3. **Mover el dron aleatoriamente**:
   Para mover el dron de forma aleatoria, ejecuta:

   ```bash
   ros2 run drone_sim drone_mover
   ```

## Ejecución del multitracker

1. **Lanzar el nodo tracker**:
   El nodo `tracker` del paquete `multitracker` utiliza un filtro de Kalman para predecir y actualizar el estado de las entidades trackeadas. Para ejecutar este nodo, utiliza:

   ```bash
   ros2 run multitracker tracker
   ```

   Este nodo actualmente solo realiza el seguimiento mediante ADS-B del cubo 1.

2. **Visualización en RViz2**:
   Los datos de seguimiento de los cubos se publican en el tópico `/cube/marker` en formato **Marker** o **MarkerArray**, y pueden ser visualizados en **RViz2**.

## Publicaciones de tópicos importantes

- **/drone/state**: Publica el estado del dron.
- **/cube/state**: Publica el estado de los cubos.
- **/drone/laser/scan**: Si se utiliza LIDAR, publica los datos del sensor en formato **PointCloud**.
- **/cube/marker**: Publica los trackeos en formato **Marker** para su visualización en RViz2.

## Mensajes personalizados

El proyecto utiliza un mensaje personalizado **Adsb.msg** que contiene el estado global de los cubos y el dron. Este mensaje se utiliza tanto para la simulación como para el trackeo de las entidades.

## Resumen

Este proyecto de simulación de seguimiento utiliza un dron con LIDAR para rastrear entidades (cubos) en un entorno de simulación de Gazebo. Los estados de los cubos se publican utilizando mensajes ADS-B y se procesan mediante un filtro de Kalman en el nodo multitracker.