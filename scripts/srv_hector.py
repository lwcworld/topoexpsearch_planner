#!/usr/bin/env python
from Params import *
from Utils_EST import *
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from topoexpsearch_planner.srv import get_path_hector
from Utils_hector import *

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

        # start index in array
        start_array = convert_cart2array((msg.start.x, msg.start.y), res, map_origin)
        print((msg.start.x, msg.start.y))
        # processing transfrom
        print('frontier calculating')
        frontier_trans_array = get_frontier_trans_array(map, self.p['min_dist_frontier_obs'], res)
        print('OT calculating')
        obs_trans_array = get_obstacle_trans_array(map)
        frontier_value_trans = get_frontier_zerovalue_trans_array(map, frontier_trans_array)
        print('EST calculating')
        exp_search_trans_array = get_EST_array(map,
                                               frontier_value_trans,
                                               obs_trans_array,
                                               self.p['alpha'],
                                               self.p['beta'],
                                               self.p['d_opt'],
                                               res)
        print('path calculating')
        path_array = get_path_gradientdescent_array(exp_search_trans_array, start_array)
        print('converting path from array to cart')
        print(path_array[0], start_array)
        path_cart = convert_path_array2cart(path_array, res, origin_array=start_array, start_cart=(msg.start.x, msg.start.y))
        print(path_cart[0], path_cart[-1])

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
    rospy.init_node('hector_server')
    print('start hector server')
    cb_srv = cb_srv()

    s = rospy.Service('get_hector', get_path_hector, cb_srv.handle_GetEST)
    rospy.spin()


