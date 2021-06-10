
# About
- provides service node which return exploratory-search path 

# tested environtment
- ubuntu 18.04
- python 2.7
- ROS melodic
- gazebo 9

# Node
## Service 1 (path_EST_server)
### input arguments
- map (nav_msgs/OccupancyGrid)
- node_values (sensor_msgs/PointCloud)
- start (in pixel) (geometry_msgs/Point32)

### return
- path (nav_msgs/Path)

## Service 2 (path_GEST_server)
### input arguments
- map (nav_msgs/OccupancyGrid)
- G_json_str (std_msgs/String)
- start (in cartesian) (geometry_msgs/Point32)

### return
- path (nav_msgs/Path)

# How to run
- EST server \
``` rosrun topoexpsearch_planner srv_EST.py ```

- server call example \
``` call_EST.py ```
