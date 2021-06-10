#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from Utils_EST import *

class Subscribers():
    def __init__(self, q_t, q_m, q_r, q_h, q_f, p):
        self.q_t = q_t
        self.q_m = q_m
        self.q_r = q_r
        self.q_h = q_h
        self.q_f = q_f
        self.p = p
        rospy.Subscriber('/hector_exploration_node/exploration_transform', PointCloud, self.cb_et, queue_size=5)
        rospy.Subscriber('/hector_exploration_node/obstacle_transform', PointCloud, self.cb_ot, queue_size=5)
        rospy.Subscriber('/exploration_path', Path, self.cb_p, queue_size=5)
        rospy.Subscriber('/odom', Odometry, self.cb_o, queue_size=5)
        rospy.Subscriber('/map', OccupancyGrid, self.cb_m, queue_size=5)

    def cb_m(self, msg):
        if self.q_f['p_map'] == False:
            self.q_t['map'] = msg.header.stamp.to_sec()
            h = msg.info.height
            w = msg.info.width
            self.q_m['map'] = np.reshape(msg.data, (h,w))
            self.q_m['res'] = msg.info.resolution
            self.q_m['origin'] = (msg.info.origin.position.x, msg.info.origin.position.y)

            self.q_m['map_msg'] = msg
            self.q_f['p_map'] = True
            print('cb : map')

    def cb_ot(self, msg):
        # callback : /exploration_transform
        self.q_t['ot'] = msg.header.stamp.to_sec()
        self.q_f['p_ot'] = False

        X = [p.x for p in msg.points]
        Y = [p.y for p in msg.points]
        Z = [p.z for p in msg.points]

        self.q_h['g_ot_pcl'][0] = X
        self.q_h['g_ot_pcl'][1] = Y
        self.q_h['g_ot_pcl'][2] = Z

        self.q_f['p_ot'] = True
        print('cb : obstacle_transform')

    def cb_et(self, msg):
        # callback : /exploration_transform
        self.q_t['et'] = msg.header.stamp.to_sec()
        self.q_f['p_et'] = False

        X = [p.x for p in msg.points]
        Y = [p.y for p in msg.points]
        Z = [p.z for p in msg.points]

        # # get frontier data
        i_zero_list = [i for i, v in enumerate(Z) if v == 10.0]
        X_f = [X[i] for i in i_zero_list]
        Y_f = [Y[i] for i in i_zero_list]

        self.q_h['g_et_pcl'][0] = X
        self.q_h['g_et_pcl'][1] = Y
        self.q_h['g_et_pcl'][2] = Z

        self.q_h['f_pcl_local'][0] = X_f
        self.q_h['f_pcl_local'][1] = Y_f
        self.q_f['p_et'] = True
        print('cb : exploration_transform')

    def cb_p(self, msg):
        self.q_t['path_hector'] = msg.header.stamp.to_sec()
        X_p = [pose.pose.position.x for pose in msg.poses]
        Y_p = [pose.pose.position.y for pose in msg.poses]
        Z_p = [pose.pose.position.z for pose in msg.poses]

        self.q_h['path'][0] = X_p
        self.q_h['path'][1] = Y_p
        self.q_h['path'][2] = Z_p

        print('cb : exploration_path')

    def cb_o(self, msg):
        # callback : /odom
        self.q_t['odom'] = msg.header.stamp.to_sec()
        P = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        self.q_r['pos'] = P
