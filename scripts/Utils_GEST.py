import networkx as nx
import math
import matplotlib.pyplot as plt
import copy
import numpy as np

def reconstruct_graph(G_json_str):
    G_json = eval(G_json_str)
    G = nx.from_edgelist(G_json["edges"])
    attr_dict = dict()
    for i_n in range(len(list(G.nodes()))):
        attr_dict[i_n] = {}
    attrs = ['isrobot', 'value', 'pos', 'to_go']
    for attr in attrs:
        for k, v in G_json[attr].items():
            attr_dict[k][attr] = v
    nx.set_node_attributes(G, attr_dict)
    return G

def get_EST_graph(G, alpha, beta):
    N_f = [n for n, attr in G.nodes(data=True) if attr['to_go'] == True] # frontier nodes

    for n, attr in G.nodes(data=True):
        G.nodes[n]['EST'] = 1000000.0

    for n in N_f:
        G.nodes[n]['EST'] = beta*(1 - G.nodes[n]['value'])
    # alpha = 0.001
    while len(N_f) > 0:
        n = N_f.pop(0)
        adj = list(G.neighbors(n))
        for k in adj:
            # k : neighrbor node
            pos_n = G.nodes[n]['pos']
            pos_k = G.nodes[k]['pos']
            dist_n_k = math.sqrt((pos_n[0]-pos_k[0])**2+(pos_n[1]-pos_k[1])**2)
            if G.nodes[k]['EST'] > G.nodes[n]['EST']  + alpha*dist_n_k:
                G.nodes[k]['EST'] = G.nodes[n]['EST']  + alpha*dist_n_k
                N_f.append(k)
    # pos = nx.get_node_attributes(G, 'pos')
    # labels = nx.get_node_attributes(G, 'EST')
    # nx.draw(G, pos, labels=labels)
    # plt.show()
    return G

def get_path_gradientdescent_graph(G):
    n = [n for n, attr in G.nodes(data=True) if attr['isrobot'] == True][0]  # robot nodes
    poses = nx.get_node_attributes(G,'pos')
    path_node = [n]
    path_pos = [poses[n]]
    while G.nodes[n]['to_go']==False:
        EST_n = G.nodes[n]['EST']
        adj = list(G.neighbors(n))
        for k in adj:
            if G.nodes[k]['EST'] < EST_n:
                n = k
        path_node.append(n)
        path_pos.append(G.nodes[n]['pos'])
    return path_node, path_pos

def convert_cart2array(pos_cart, res, map_origin):
    # position of point in cartesian coordinate
    # res : map resolution
    # o : map origin
    x_cart = pos_cart[0]
    y_cart = pos_cart[1]
    i_array = int((y_cart - map_origin[1])/res)
    j_array = int((x_cart - map_origin[0])/res)
    return (i_array, j_array)

def convert_path_array2cart(path_array, res, origin_array, start_cart):
    path_cart = []
    for point_array in path_array:
        i_array = point_array[0]
        j_array = point_array[1]
        x_cart = (j_array - origin_array[0]) * res + start_cart[0]
        y_cart = (i_array - origin_array[1]) * res + start_cart[1]
        point_cart = (x_cart, y_cart)
        path_cart.append(point_cart)
    return path_cart


def get_local_map_arbitrary(glo_map, glo_pos, r=20):
    (x, y) = glo_pos
    lx, ux = max(0, x - r - 1), min(glo_map.shape[1], x + r + 1 + 1)
    ly, uy = max(0, y - r - 1), min(glo_map.shape[0], y + r + 1 + 1)

    loc_map = copy.deepcopy(glo_map[ly:uy, lx:ux])
    x = min(x, r+1)
    y = min(y, r+1)
    for j in range(loc_map.shape[0]):
        for i in range(loc_map.shape[1]):
            dist = get_dist((x, y), (i, j))
            if (dist <= r):
                if not visible(loc_map, (x, y), (i, j)) and loc_map[j,i]!=0:
                    loc_map[j, i] = 0.5
            else:
                loc_map[j, i] = 0.5
    return loc_map, (x, y)

def get_local_map(glo_map, glo_pos, r=100):
    (x, y) = glo_pos
    lx, ux = max(0, x - r - 1), min(glo_map.shape[1], x + r + 1 + 1)
    ly, uy = max(0, y - r - 1), min(glo_map.shape[0], y + r + 1 + 1)

    # plt.imshow(glo_map)
    # plt.show()
    loc_map = copy.deepcopy(glo_map[ly:uy, lx:ux])

    # plt.imshow(loc_map)
    # plt.show()

    return loc_map

def get_frontier_zerovalue_trans_array(local_map, frontier_trans_array):
    idx_frontiers = np.where(frontier_trans_array==1)
    frontier_value_trans = np.ones(np.shape(local_map))*(-1)
    frontier_value_trans[idx_frontiers[0], idx_frontiers[1]] = 0
    return frontier_value_trans

def get_straight_idxs(i_s):
    i = i_s[0]
    j = i_s[1]
    return [(i+1,j), (i-1,j), (i,j+1), (i,j-1)]

def get_diagonal_idxs(i_s):
    i = i_s[0]
    j = i_s[1]
    return [(i+1,j+1), (i-1,j-1), (i-1,j+1), (i+1,j-1)]

def get_path_gradientdescent_array_from_localmap(costmap, idx_start, res, start_cart):
    flag_arrive = False
    idx = copy.deepcopy(idx_start)
    path_array_local = [idx]
    while flag_arrive==False:
        adj_s = get_straight_idxs(idx)
        adj_d = get_diagonal_idxs(idx)
        adj_list = adj_s + adj_d
        min_cost = costmap[idx[0], idx[1]]
        for adj in adj_list:
            if costmap[adj[0], adj[1]] < min_cost:
                min_cost = costmap[adj[0], adj[1]]
                min_adj = adj
        if min_cost == costmap[idx[0], idx[1]] or min_cost==0:
            flag_arrive = True
        else:
            path_array_local.append(min_adj)
            idx = min_adj

    path_cart = convert_path_array2cart(path_array_local, res, origin_array=idx_start, start_cart=start_cart)

    return path_cart


def get_frontier_trans_array(map, min_dist_frontier_obs, res):
    idx_free = np.where(map == 0)
    frontier_trans = np.zeros(np.shape(map))
    d_cell = int(min_dist_frontier_obs/res)
    for i, j in zip(idx_free[0], idx_free[1]):
        check_unknown = [map[i-1,j]==-1, map[i+1,j]==-1 , map[i,j-1]==-1 , map[i,j+1]==-1 , map[i-1,j+1]==-1 , map[i-1,j-1]==-1 , map[i+1,j+1]==-1, map[i+1,j-1]==-1]
        check_occ = (map[i-d_cell:i+d_cell+1,j-d_cell:j+d_cell+1] == 100)
        if check_unknown.count(True)>2 and np.all(check_occ==False):
            frontier_trans[i,j] = 1

    idx_frontier = np.where(frontier_trans==1)
    for i, j in zip(idx_frontier[0], idx_frontier[1]):
        check_frontier = [frontier_trans[i - 1, j] == 1,
                          frontier_trans[i + 1, j] == 1,
                          frontier_trans[i, j - 1] == 1,
                          frontier_trans[i, j + 1] == 1,
                          frontier_trans[i - 1, j + 1] == 1,
                          frontier_trans[i - 1, j - 1] == 1,
                          frontier_trans[i + 1, j + 1] == 1,
                          frontier_trans[i + 1, j - 1] == 1]
        if check_frontier.count(True) < 1:
            frontier_trans[i,j] = 0
    return frontier_trans


def get_obstacle_trans_array(map):
    idx_obs = np.where(map==100)
    queue_i = list(idx_obs[0])
    queue_j = list(idx_obs[1])
    obs_trans = np.ones(np.shape(map))*100000.0
    obs_trans[idx_obs[0], idx_obs[1]] = 0.0
    while len(queue_i) > 0:
        i = queue_i.pop(0)
        j = queue_j.pop(0)
        adj_s = get_straight_idxs((i,j))
        adj_d = get_diagonal_idxs((i,j))
        for k in range(4):
            if map[adj_s[k][0], adj_s[k][1]]==0 and (obs_trans[i,j] + 1.0 < obs_trans[adj_s[k][0], adj_s[k][1]]):
                obs_trans[adj_s[k][0], adj_s[k][1]] = obs_trans[i,j] + 1.0
                queue_i.append(adj_s[k][0])
                queue_j.append(adj_s[k][1])
            if map[adj_d[k][0], adj_d[k][1]]==0 and (obs_trans[i,j] + 1.414 < obs_trans[adj_d[k][0], adj_d[k][1]]):
                obs_trans[adj_d[k][0], adj_d[k][1]] = obs_trans[i,j] + 1.414
                queue_i.append(adj_d[k][0])
                queue_j.append(adj_d[k][1])
    idx_dummy = np.where(obs_trans==100000.0)
    obs_trans[idx_dummy[0], idx_dummy[1]] = 0
    return obs_trans


def get_EST_array(map, frontier_value_trans_array, obs_trans_array, alpha, beta, d_opt, res):
    idx_frontier = np.where(frontier_value_trans_array!=-1)
    queue_i = list(idx_frontier[0])
    queue_j = list(idx_frontier[1])
    exp_trans = np.ones(np.shape(map))*1000000.0
    exp_trans[idx_frontier[0], idx_frontier[1]] = beta*(1 - frontier_value_trans_array[idx_frontier[0], idx_frontier[1]])
    while len(queue_i) > 0:
        i = queue_i.pop(0)
        j = queue_j.pop(0)
        adj_s = get_straight_idxs((i,j))
        adj_d = get_diagonal_idxs((i,j))
        for k in range(4):
            if map[adj_s[k][0], adj_s[k][1]]==0 and (exp_trans[i,j] + 1 + alpha*(c_danger(obs_trans_array,(adj_s[k][0],adj_s[k][1]),d_opt,res)) < exp_trans[adj_s[k][0], adj_s[k][1]]):
                exp_trans[adj_s[k][0], adj_s[k][1]] = exp_trans[i,j] + 1 + alpha*(c_danger(obs_trans_array,(adj_s[k][0],adj_s[k][1]),d_opt,res))
                queue_i.append(adj_s[k][0])
                queue_j.append(adj_s[k][1])
            if map[adj_d[k][0], adj_d[k][1]]==0 and (exp_trans[i,j] + 1.414 + alpha*(c_danger(obs_trans_array,(adj_d[k][0],adj_d[k][1]),d_opt,res)) < exp_trans[adj_d[k][0], adj_d[k][1]]):
                exp_trans[adj_d[k][0], adj_d[k][1]] = exp_trans[i,j] + 1.414 + alpha*(c_danger(obs_trans_array,(adj_d[k][0],adj_d[k][1]),d_opt,res))
                queue_i.append(adj_d[k][0])
                queue_j.append(adj_d[k][1])
    idx_dummy = np.where(exp_trans==1000000.0)
    exp_trans[idx_dummy[0], idx_dummy[1]] = 0
    return exp_trans