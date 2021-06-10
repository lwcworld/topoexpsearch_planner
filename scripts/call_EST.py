#!/usr/bin/env python
from Params import *
from Variables import *
from Subscribers import *
import matplotlib.pyplot as plt
import networkx as nx
from topoexpsearch_planner.srv import get_ET
from geometry_msgs.msg import Point32

if __name__ == '__main__':
    rospy.init_node('test_ET')

    # declare classes
    q_t = time_stamp() # time stamp
    q_r = robot()
    q_h = hector() # hector info
    q_p = plan()
    q_f = flag()
    q_m = map()
    p = param()

    # declare subscriber
    sub = Subscribers(q_t=q_t, q_m=q_m, q_r=q_r, q_h=q_h, q_f=q_f, p=p)

    # service
    rospy.wait_for_service('get_ET')
    srv_get_ET = rospy.ServiceProxy('get_ET', get_ET)

    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        if q_f['p_map'] == True:
            # map
            map = q_m['map_msg']

            map = np.reshape(map.data, (q_m['map_msg'].info.height, q_m['map_msg'].info.width))

            # toy node data
            NN = nx.Graph()
            NN.add_nodes_from([(0, {'pos': (-28., 0.), 'type': 3, 'is_robot': True, 'to_go': False, 'value': 0}),
                               (1, {'pos': (-35., 0.), 'type': 2, 'is_robot': False, 'to_go': True, 'value': 0.1}),
                               (2, {'pos': (-28., 5.), 'type': 1, 'is_robot': False, 'to_go': True, 'value': 0.9}),
                               (3, {'pos': (-28., -5.), 'type': 2, 'is_robot': False, 'to_go': True, 'value': 0.1}),
                               (4, {'pos': (-21., 0.), 'type': 1, 'is_robot': False, 'to_go': True, 'value': 0.1})])
            NN.add_edges_from([(0, 1), (0, 2), (0, 3), (0, 4)])

            # node message construction
            node_values = PointCloud()
            node_values.header.stamp =  rospy.Time.now()
            node_values.header.frame_id = "map"
            node_poses_dict = get_nx_node_pos_array(NN, q_m['res'], q_m['origin'])
            node_values_dict = nx.get_node_attributes(NN, 'value')

            for pos, value in zip(node_poses_dict.values(), node_values_dict.values()):
                point = Point32()
                point.x = pos[0]
                point.y = pos[1]
                point.z = value
                node_values.points.append(point)

            start = Point32()
            start.x = 400
            start.y = 400

            output = srv_get_ET(map, node_values, start)
            path = output.path
            i_path = [point.pose.position.x for point in path.poses]
            j_path = [point.pose.position.y for point in path.poses]
            plt.scatter(j_path, i_path, s=0.5, c='green')
            plt.show()

            q_f['p_map'] = False
            break

        rate.sleep()