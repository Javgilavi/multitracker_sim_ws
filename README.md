# Multitracker Simulation Project

This project is developed for **ROS2 Humble** using **GAZEBO Classic** on **Ubuntu 22.04**. The project consists of three packages within `src`, which are used to simulate an entity tracking environment through sensor fusion using a LIDAR and ADS-B data.

## Project Structure

- **drone_sim**: Contains the Gazebo simulation environment, along with complementary nodes to move the drone and the cubes.
- **multitracker**: Node that acts as a multitracker for entities, using sensor data.
- **sim_msgs**: Package that contains the custom messages used in the project, such as `Adsb.msg`.

## General Overview

This project simulates a drone equipped with a **LIDAR** sensor that tracks three entities (cubes) through sensor data fusion. The cubes periodically publish their states via **ADS-B** messages on their respective topics, allowing the drone to track them in real-time.

- The drone publishes its state on the `/drone/state` topic.
- The cubes publish their state on the `/cube/state` topic.
- If the LIDAR is used, the sensor will publish **PointCloud** messages on the `/drone/laser/scan` topic.

The **multitracker** node uses a Kalman filter to predict and update the state of the entities, and the tracking data is visualized in RViz2 through the `/cube/marker` topic.

## Requirements

- **ROS2 Humble**
- **Gazebo Classic**
- **Ubuntu 22.04**

## Steps for the Training

1. **Transform ADS-B tracks from global to local**
2. **Track Cube1 from its ADS-B data using Kalman Filter**
3. **Create a list to track all 3 cubes at the same time**
4. **strengthen the multitracker (random dissaparence, low-pass filter, ...)**
5. **Add LIDAR measurement using a driver to get tracks**
6. **Matching of LIDAR tracks with obstacle in the list**
7. **New matrix in Kalman Filter for LIDAR tracks**
8. **Final testings**

## Compilation Instructions

1. **Build the project**:
   First, build the `sim_msgs` package, followed by the rest of the project.

   ```bash
   colcon build --packages-select sim_msgs
   colcon build
   ```

2. **Source the environment**:
   After building, execute:

   ```bash
   source install/setup.bash
   ```

## Running the Simulation in Gazebo

1. **Launch the simulation**:
   To start the Gazebo simulation environment, use the following command:

   ```bash
   ros2 launch drone_sim drone_world_launch.py
   ```

   In the launch file <drone_sim/launch/drone_world_launch.py>, on line 16, you can change `model.sdf` to `model_lidar.sdf` to include or exclude the LIDAR sensor on the drone.

2. **Move the cubes randomly**:
   To move the cubes randomly in the simulation, execute:

   ```bash
   ros2 run drone_sim cube_mover
   ```

3. **Move the drone randomly**:
   To move the drone randomly, execute:

   ```bash
   ros2 run drone_sim drone_mover
   ```

## Running the Multitracker

1. **Launch the tracker node**:
   The `tracker` node from the `multitracker` package uses a Kalman filter to predict and update the state of the tracked entities. To run this node, use:

   ```bash
   ros2 run multitracker tracker
   ```

   The tracking data for the cubes is published on the `/cube/marker` topic in **Marker** or **MarkerArray** format, and can be visualized in **RViz2**.

   You can also launch both the `tracker` node with **RViz2**, with its adapted configuration, using:

   ```bash
   ros2 launch multitracker tracker_launch.py
   ```

2. **Error visualization**:
When you stop the `multitracker` node with `CTRL+C`, the **RMSE** of position and orientation for each tracked obstacle will be displayed. Additionally, the error data will be saved in `error_data.txt` in the workspace.

To visualize the errors, run the Python script:

```bash
python3 /home/flyros/multitracker_sim_ws/error_plotter.py
```

This script generates graphs for positional and orientational errors for each tracked obstacle.

> [!NOTE]
> This node currently only **transform from global to local** the Cube1 ADS-B data.

## Important Topics

- **/drone/state**: Publishes the state of the drone.
- **/cube/state**: Publishes the state of the cubes.
- **/drone/laser/scan**: If the LIDAR is used, it publishes the sensor data in **PointCloud** format.
- **/cube/marker**: Publishes the tracking data in **Marker** format for visualization in RViz2.

## Custom Messages

The project uses a custom **Adsb.msg** message that contains the global state of the cubes and the drone. This message is used both for the simulation and for tracking the entities.

## Summary

This simulation project uses a drone equipped with LIDAR to track entities (cubes) in a Gazebo simulation environment. The states of the cubes are published using ADS-B messages and are processed by a Kalman filter in the multitracker node.