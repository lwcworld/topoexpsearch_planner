
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
```
example: {'to_go': {0: False, 1: True, 2: False, 3: False, 4: False, 5: False, 6: False, 7: False, 8: False, 9: False}, 'edges': [(0, 9), (1, 2), (2, 8), (2, 4), (3, 9), (4, 9), (5, 6), (6, 9), (7, 9)], 'pos': {0: (-19.743748804088682, 6.600001588463783), 1: (3.9232158342908536, 1.8535729463079147), 2: (-1.691665201758345, 3.095834869580964), 3: (-13.67499871365726, 6.712501590140164), 4: (-6.9874986140057445, 3.4937515421770513), 5: (-16.796427331598743, -5.060712871008685), 6: (-17.820832108768325, -3.770831899407009), 7: (-11.389284393883177, -4.914284297398159), 8: (1.0062515051104128, -5.237498587928712), 9: (-18.004165444833536, 0.8972237257079946)}, 'value': {0: 0.3, 1: 0.3, 2: 0.3, 3: 0.3, 4: 0.3, 5: 0.3, 6: 0.3, 7: 0.3, 8: 0.3, 9: 0.3}, 'isrobot': {0: False, 1: False, 2: False, 3: False, 4: False, 5: False, 6: False, 7: False, 8: False, 9: True}}
```

- start (in cartesian) (geometry_msgs/Point32)

### return
- path (nav_msgs/Path)

# How to run
- EST server \
``` rosrun topoexpsearch_planner srv_EST.py ```

- server call example \
``` call_EST.py ```
