#!/usr/bin/env python
from Params import *
from Utils_EST import *
from Utils_GEST import  *
from pytictoc import TicToc
import numpy as np
import cv2
import matplotlib.pyplot as plt
import json

# ===== toy inputs =====
# map related
q_m = np.load('../data/q_m.npy', allow_pickle='TRUE').item()
q_r = np.load('../data/q_r.npy', allow_pickle='TRUE').item()

# ============================
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

# plt.imshow(obs_trans_array)
# plt.show()

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

# =========================
map = q_m['map_2d']
map_gray = np.array(map * 255 / np.max(map), dtype=np.uint8)
map_rgb = cv2.cvtColor(map_gray, cv2.COLOR_GRAY2BGR)

# graph
node_pos_dict = nx.get_node_attributes(q_m['global_network'], 'pos')

for node, node_pos in node_pos_dict.items():
    node_pos_dict[node] = convert_cart2array(node_pos, q_m['res'], q_m['origin'])

# edges
e_list = q_m['global_network'].edges()
for e in e_list:
    pos_0 = (node_pos_dict[e[0]][1], node_pos_dict[e[0]][0])
    pos_1 = (node_pos_dict[e[1]][1], node_pos_dict[e[1]][0])
    map_rgb = cv2.line(map_rgb, pos_0, pos_1, (0, 100, 100), 2)
# nodes
for node, pos in node_pos_dict.items():
    map_rgb = cv2.circle(map_rgb, (pos[1], pos[0]), 7, (0, 255, 0), cv2.FILLED)


for point in path_cart:
    point_array = convert_cart2array(point, q_m['res'], q_m['origin'])
    map_rgb = cv2.circle(map_rgb, (point_array[1], point_array[0]), 7, (0, 255, 255), cv2.FILLED)

map_rgb = cv2.flip(map_rgb, 0)

plt.imsave('test.png', map_rgb, dpi=600, cmap=plt.cm.Reds)