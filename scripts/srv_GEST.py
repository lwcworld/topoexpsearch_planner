#!/usr/bin/env python
from Params import *
from Utils_GEST import  *
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from topoexpsearch_planner.srv import get_path_GEST

class cb_srv():
    def __init__(self):
        self.p = param()

    def handle_GetGEST(self, msg):
        print('received call')
        # ===== parse map msg =====
        res = msg.map.info.resolution
        h = msg.map.info.height
        w = msg.map.info.width
        map_origin = (msg.map.info.origin.position.x, msg.map.info.origin.position.y)
        map = np.reshape(msg.map.data, (h,w))


        # ===== parse graph msg =====
        G_json_str = msg.G_json_str
        G = reconstruct_graph(G_json_str)
        # ===== get graph EST =====
        G = get_EST_graph(G, self.p['alpha'], self.p['beta'])
        # ===== get graph path =====
        path_graph_node, path_graph_pos = get_path_gradientdescent_graph(G)

        # ===== get last pos of graph path =====
        last_pos_path_graph = path_graph_pos[-1]
        (i_array, j_array) = convert_cart2array(last_pos_path_graph, res, map_origin)

        # ===== get local map =====
        local_map = get_local_map(map, (i_array, j_array), r=200)

        # ===== get frontiers =====
        frontier_trans_array = get_frontier_trans_array(local_map, self.p['min_dist_frontier_obs'], res)

        # ===== get obstacle transform =====
        obs_trans_array = get_obstacle_trans_array(local_map)

        # ===== get exploration transform =====
        frontier_value_trans = get_frontier_zerovalue_trans_array(local_map, frontier_trans_array)
        EST_local = get_EST_array(local_map,
                                  frontier_value_trans,
                                  obs_trans_array,
                                  self.p['alpha'],
                                  self.p['beta'],
                                  self.p['d_opt'],
                                  res)

        path_local_cart = get_path_gradientdescent_array_from_localmap(EST_local, (200, 200), res, path_graph_pos[-1])
        path_cart = path_graph_pos + path_local_cart

        # convert to Path topic
        path_topic = Path()
        path_topic.header.stamp = rospy.Time.now()
        path_topic.header.frame_id = "map"
        for pos in path_cart:
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

    s = rospy.Service('get_ET', get_path_GEST, cb_srv.handle_GetGEST)
    rospy.spin()


