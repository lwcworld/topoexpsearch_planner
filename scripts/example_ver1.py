#!/usr/bin/env python
from Params import *
from Utils_EST import *
from pytictoc import TicToc
import numpy as np
import cv2
import matplotlib.pyplot as plt

# ===== toy inputs =====
# map related
q_m = np.load('../data/q_m.npy', allow_pickle='TRUE').item()
q_r = np.load('../data/q_r.npy', allow_pickle='TRUE').item()

# nodes pos
node_pos_dict = get_nx_node_pos_array(q_m['global_network'], q_m['res'], q_m['origin'])
# nodes value
for i_n in range(len(q_m['global_network'].nodes)):
    q_m['global_network'].nodes[i_n]['value'] = 0.3
q_m['global_network'].nodes[2]['value'] = 0.9
node_value_dict = nx.get_node_attributes(q_m['global_network'], 'value')

# ===== param set =====
p = param()

t = TicToc()
# ===== get frontiers =====
print('get frontiers')
t.tic()
frontier_trans_array = get_frontier_trans_array(q_m['map_2d'], p['min_dist_frontier_obs'], q_m['res'])
t.toc()

# ===== get obstacle transform =====
print('get OT')
t.tic()
obs_trans_array = get_obstacle_trans_array(q_m['map_2d'])
t.toc()

# ===== frontier value calculation =====
print('get frontier value')
t.tic()
frontier_value_trans = get_frontier_value_trans_array(node_pos_dict, node_value_dict, q_m['map_2d'],
                                                      frontier_trans_array)
print(frontier_value_trans)
print(np.unique(frontier_value_trans))
t.toc()

# ===== get exploration transform =====
print('get EST')
t.tic()
EST = get_EST_array(q_m['map_2d'], frontier_value_trans, obs_trans_array, p['alpha'],
                                             p['beta'], p['d_opt'], q_m['res'])
t.toc()


# ==========================================
# ===== get path =====
print('get path')
t.tic()
path = get_path_gradientdescent_array(EST, (q_r['pos_pix'][1], q_r['pos_pix'][0]))  # path
t.toc()

# ==== plot OT & graph ====
frontiers = np.where(frontier_trans_array == 1)
OT_gray = np.array(obs_trans_array * 255 / np.max(obs_trans_array), dtype=np.uint8)
OT_rgb = cv2.cvtColor(OT_gray, cv2.COLOR_GRAY2BGR)
# frontier
for x, y in zip(frontiers[0], frontiers[1]):
    OT_rgb = cv2.circle(OT_rgb, (y, x), 2, (255, 0, 0), cv2.FILLED)
# edges
e_list = q_m['global_network'].edges()
for e in e_list:
    pos_0 = (node_pos_dict[e[0]][1], node_pos_dict[e[0]][0])
    pos_1 = (node_pos_dict[e[1]][1], node_pos_dict[e[1]][0])
    OT_rgb = cv2.line(OT_rgb, pos_0, pos_1, (0, 100, 100), 2)
# nodes
for node, pos in node_pos_dict.items():
    OT_rgb = cv2.circle(OT_rgb, (pos[1], pos[0]), int(node_value_dict[node] * 10), (0, 255, 0), cv2.FILLED)
OT_rgb = cv2.flip(OT_rgb, 0)
plt.imsave('OT.png', OT_rgb, dpi=600, cmap=plt.cm.Reds)

pos = nx.get_node_attributes(q_m['global_network'], 'pos')
nx.draw(q_m['global_network'], pos, with_labels=True)
plt.show()

pos = nx.get_node_attributes(q_m['global_network'], 'pos')
labels = nx.get_node_attributes(q_m['global_network'], 'value')
nx.draw(q_m['global_network'], pos, labels=labels)
plt.show()

# ==== plot EST & path ====
EST_gray = np.array(EST * 255 / np.max(EST), dtype=np.uint8)
EST_rgb = cv2.cvtColor(EST_gray, cv2.COLOR_GRAY2BGR)
# path
for point in path:
    EST_rgb = cv2.circle(EST_rgb, (point[1], point[0]), 2, (255, 255, 0))
# frontier
for x, y in zip(frontiers[0], frontiers[1]):
    EST_rgb = cv2.circle(EST_rgb, (y, x), 2, (255, 0, 0))

# robot
EST_rgb = cv2.circle(EST_rgb, (q_r['pos_pix'][1], q_r['pos_pix'][0]), 4, (0, 255, 0))

# invert
EST_rgb = cv2.flip(EST_rgb, 0)
plt.imsave('EST_path.png', EST_rgb, dpi=600, cmap=plt.cm.Reds)

print('done')