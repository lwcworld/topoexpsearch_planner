#!/usr/bin/env python
from Params import *
from Utils_EST import *
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from topoexpsearch_planner.srv import get_path_EST

class cb_srv():
    def __init__(self):
        self.p = param()

    def handle_GetEST(self, msg):
        print('received call')
        # map datas
        res = msg.map.info.resolution
        h = msg.map.info.height
        w = msg.map.info.width
        map = np.reshape(msg.map.data, (h,w))
        map_origin = (msg.map.info.origin.position.x, msg.map.info.origin.position.y)

        # node datas
        node_values = msg.node_values
        i_list = [p.x for p in node_values.points]
        j_list = [p.y for p in node_values.points]
        v_list = [p.z for p in node_values.points]
        node_pos_dict = {}
        node_value_dict = {}
        for iter, (i,j,v) in enumerate(zip(i_list, j_list, v_list)):
            node_pos_dict[iter] = (int(i),int(j))
            node_value_dict[iter] = v

        # start index in array
        start = convert_cart2array((msg.start.x, msg.start.y), res, map_origin)

        # processing transfrom
        frontier_trans_array = get_frontier_trans_array(map, self.p['min_dist_frontier_obs'], res)
        obs_trans_array = get_obstacle_trans_array(map)
        frontier_value_trans = get_frontier_value_trans_array(node_pos_dict, node_value_dict, map, frontier_trans_array)
        exp_search_trans_array = get_EST_array(map,
                                               frontier_value_trans,
                                               obs_trans_array,
                                               self.p['alpha'],
                                               self.p['beta'],
                                               self.p['d_opt'],
                                               res)
        path = get_path_gradientdescent_array(exp_search_trans_array, start)

        # convert to Path topic
        path_topic = Path()
        path_topic.header.stamp = rospy.Time.now()
        path_topic.header.frame_id = "map"
        for pos in path:
            point = PoseStamped()
            point.header.stamp = rospy.Time.now()
            point.header.frame_id = "map"
            point.pose.position.x = pos[0]
            point.pose.position.y = pos[1]
            point.pose.position.z = 0
            path_topic.poses.append(point)
        return path_topic

if __name__ == '__main__':
    rospy.init_node('EST_server')
    print('start EST server')
    cb_srv = cb_srv()

    s = rospy.Service('get_ET', get_path_EST, cb_srv.handle_GetEST)
    rospy.spin()


