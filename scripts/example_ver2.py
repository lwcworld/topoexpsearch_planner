#!/usr/bin/env python
from Params import *
# from Utils_EST import *
from Utils_GEST import  *
from pytictoc import TicToc
import numpy as np
import cv2
import matplotlib.pyplot as plt
import json

# ===== caller =====
# map related
q_m = np.load('../data/q_m.npy', allow_pickle='TRUE').item()
q_r = np.load('../data/q_r.npy', allow_pickle='TRUE').item()

# remove self recurisve edges
edges = list(q_m['global_network'].edges())
for edge in edges:
    if edge[0] == edge[1]:
        q_m['global_network'].remove_edge(*edge)

# nodes to_go
Frontier_nodes = [1]
for i_n in range(len(q_m['global_network'].nodes)):
    if i_n in Frontier_nodes:
        q_m['global_network'].nodes[i_n]['to_go'] = True
    else:
        q_m['global_network'].nodes[i_n]['to_go'] = False

# nodes value
for i_n in range(len(q_m['global_network'].nodes)):
    q_m['global_network'].nodes[i_n]['value'] = 0.3

# nodes pos (caller)
edges  = list(q_m['global_network'].edges())
value = nx.get_node_attributes(q_m['global_network'], 'value')
pos = nx.get_node_attributes(q_m['global_network'], 'pos')
isrobot = nx.get_node_attributes(q_m['global_network'], 'isrobot')
to_go = nx.get_node_attributes(q_m['global_network'], 'to_go')

G_json = {"edges":edges, "value":value, "isrobot":isrobot, "pos":pos, "to_go":to_go}
G_json_str = str(G_json)

map_origin = q_m['origin']
res = q_m['res']

# ===== service part =====
# <inputs>
# G_json_str
# map

# ===== param set =====
p = param()

# ===== parse =====
G = reconstruct_graph(G_json_str)
# pos = nx.get_node_attributes(G, 'pos')
# nx.draw(G, pos, with_labels=True)
# plt.show()
# ===== get graph EST =====
G = get_EST_graph(G, p['alpha'], p['beta'])

# ===== get graph path =====
path_graph_node, path_graph_pos = get_path_gradientdescent_graph(G)

# =====
pos_last = path_graph_pos[-1]

(i_array, j_array) = convert_cart2array(pos_last, res, map_origin)

local_map = get_local_map(q_m['map_2d'], (i_array, j_array), r=200)

# ===== get frontiers =====
frontier_trans_array = get_frontier_trans_array(local_map, p['min_dist_frontier_obs'], q_m['res'])

# # ===== get obstacle transform =====
obs_trans_array = get_obstacle_trans_array(local_map)

# ===== get exploration transform =====
frontier_value_trans = get_frontier_zerovalue_trans_array(local_map, frontier_trans_array)
EST_local = get_EST_array(local_map,
                    frontier_value_trans,
                    obs_trans_array,
                    p['alpha'],
                    p['beta'],
                    p['d_opt'],
                    q_m['res'])
path_local_cart = get_path_gradientdescent_array_from_localmap(EST_local, (200, 200), res, path_graph_pos[-1])
path_cart = path_graph_pos + path_local_cart

plt.imshow(q_m['map_2d'])
plt.show()


# plt.imshow(q_m['map_2d'])
# plt.show()
#
# path_array = []
# for point_cart in path_cart:
#     point_array = convert_cart2array(point_cart, res, map_origin)
#     path_array.append(point_array)
#
#
# x_list = []
# y_list = []
# for point_array in path_array:
#     x_list.append(point_array[0])
#     y_list.append(point_array[1])
#
# plt.imshow(EST_local)
# plt.plot(x_list, y_list, '-o')
# plt.axis('scaled')
# plt.show()
#
# import matplotlib.image as mpimg
# img = mpimg.imread('OT.png')
# plt.plot(y_list, x_list , 'r-', )
# plt.gca().invert_xaxis()
# imgplot = plt.imshow(img)
# plt.show()
# print(path_cart)

print('finished')