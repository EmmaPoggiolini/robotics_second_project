# Robotics Second Project - Mapping and Autonomous Navigation

## Code Organization

- `CMakeList.txt` this file contains a description of the project's source files and targets
- `info.txt` this file contrains the registration number, name, last name of the authors 
- `package.xml` this file contains a collection of metadata components 
- `launch/*` this folder contains the project launch files 
- `maps_raw/*` this folder contains the `.pgm` and `.yaml` files of the generated maps
- `src/navigation.cpp` this executable instantiates the `navigation` node, reads the new goal from the `waypoints.csv` file (defined in the `/navigation/csv_path` parameter) and publishes it via move base action
- `src/tf_publisher.cpp` this executable instantiates the `tf_publisher` node and publishes the tf between odom and t265 frames based on the t265 odometry
- `stage/*` this folder contains the `.pgm`, `.yaml`, `.png` and `.world` files of the final chosen map
- `config_mapping.rviz` this file contains the rviz configuration parameters for the generation of the map
- `config_nav.rviz` this file contains the rviz configuration parameters for the robot navigation 
- `waypoints.csv` this file contains the waypoints read by the `navigation` node to set the new goal

## Installation 

Prerequisites:
  - Linux environment
  - ROS noetic  

### Two nodes are implemented:

- `tf_publisher`: publishes the tf between odom and t265 frames based on the t265 odometry
- `navigation`: reads a csv file (defined in the `/navigation/csv_path` parameter) and publishes the next goals via move base action

## Generating the map:

Launchfile for generating the map with the provided single plane scanner data (from `/scan` topic):

```bash
roslaunch second_project slam_toolbox_scan.launch
```

Launchfile for generating the map with the 2D data converted from the 3D laser (includes a `pointcloud_to_laserscan` node)

```bash
roslaunch second_project slam_toolbox_2d_scan.launch
```

After launching either one of these, a rosbag should be played in order to generate the map.

Both launchfiles were run with two separate bags, and the resulting maps were stored in the `maps_raw` folder.
These commands also launch the `tf_publisher` node together with other static transforms, and RViz with the `config_mapping.rviz` configuration.

After the map was generated, it was saved by running:

```bash
rosrun map_server map_saver -f <map_name>
```

## Running the navigation:

The map chosen for navigation was the map generated from the converted 2D scanner by playing the first bag; noise was removed in post-processing.

The algorithm chosen for local path planning is DWA. 

```bash
roslaunch second_project amcl.launch
```

This also launches the `navigation` node and RViz with the `config_navigation.rviz` configuration.

The `navigation` node reads a sequence of waypoints, one at a time, from the `waypoints.csv` file; each waypoint represents the new goal that the robot attempts to reach. 

## Nodes and other packages used:

- `pointcloud_to_laserscan` to convert velodyne pointcloud to laserscan
- `slam_toolbox` with `gmapping` to generate the map (detailed config in `cfg/st_config_scan.yaml`)
- `dwa_local_planner` for local planning 
- `tf` and `tf2_ros` for static transforms
- `rviz` for visualization
- `stage_ros` for stage simulation
- `move_base` for the navigation stack
- `amcl` for localization
- `map_server` for saving/serving the map
- `amcl` for autonomous navigation
