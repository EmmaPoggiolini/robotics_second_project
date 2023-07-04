# Robotics Second Project - Mapping and autonomous navigation

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

After starting either of these, a bag should be played and the map will be generated.

Both of these have been run with both bags, and the results are in the `maps_raw` folder.
These will also launch the `tf_publisher` node together with other static transforms, and RViz with the `config_mapping.rviz` configuration.

After the map was generated, it has been saved with:

```bash
rosrun map_server map_saver -f <map_name>
```

## Running the navigation:

The map generated from the converted 2D scanner with the first bag was used for the navigation, we removed some noise in post-processing.

```bash
roslaunch second_project amcl.launch
```

This will also launch the `navigation` node and RViz with the `config_navigation.rviz` configuration.

## Nodes and other packages used:

- `pointcloud_to_laserscan` to convert velodyne pointcloud to laserscan
- `slam_toolbox` with `gmapping` to generate the map (detailed config in `cfg/st_config_scan.yaml`)
- `tf` and `tf2_ros` for static transforms
- `rviz` for visualization
- `stage_ros` for stage simulation
- `move_base` for the navigation stack
- `amcl` for localization
- `map_server` for saving/serving the map
- `amcl` for autonomous navigation
