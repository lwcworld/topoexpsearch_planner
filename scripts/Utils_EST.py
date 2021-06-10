#!/usr/bin/env python
import networkx as nx
import numpy as np
import math
import copy
import matplotlib.pyplot as plt


def get_robot_idx(O, X, Y):
    x_r = O[0]
    y_r = O[1]

    d_min = 1000.
    i_min = 0
    for i, (x,y) in enumerate(zip(X,Y)):
        d = math.sqrt((x-x_r)**2 + (y-y_r)**2)
        if d < d_min:
            d_min = d
            i_min = i
    return i_min

def get_adj_idxs(i_r, X, Y, res):
    x_r = X[i_r]
    y_r = Y[i_r]
    i_adj_list = [i for i, (x,y) in enumerate(zip(X,Y)) if  math.sqrt((x-x_r)**2 + (y-y_r)**2)<res*math.sqrt(2) and i!=i_r]
    return i_adj_list

def get_straight_idxs(i_s):
    i = i_s[0]
    j = i_s[1]
    return [(i+1,j), (i-1,j), (i,j+1), (i,j-1)]

def get_diagonal_idxs(i_s):
    i = i_s[0]
    j = i_s[1]
    return [(i+1,j+1), (i-1,j-1), (i-1,j+1), (i+1,j-1)]

def get_adj_idxs_array(i_s, map, condition):
    idx_s = get_straight_idxs(i_s)
    idx_d = get_diagonal_idxs(i_s)
    adj_list = []
    for c in range(0,4):
        if map[idx_s[c][0], idx_s[c][1]] == condition:
            adj_list.append(idx_s[c])
        if map[idx_d[c][0], idx_d[c][1]] == condition:
            adj_list.append(idx_s[c])
    return adj_list

def get_path_gradientdescent_array(costmap, idx_start):
    flag_arrive = False
    idx = copy.deepcopy(idx_start)
    path = [idx]
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
            path.append(min_adj)
            idx = min_adj
    return path

def get_gradients(i_r, i_adj_list, X, Y, C):
    x_r = X[i_r]
    y_r = Y[i_r]
    c_r = C[i_r]

    grad_list = []
    for i_adj in i_adj_list:
        x_adj = X[i_adj]
        y_adj = Y[i_adj]
        c_adj = C[i_adj]
        grad = (c_adj - c_r) / math.sqrt((x_adj-x_r)**2 + (y_adj-y_r)**2)
        grad_list.append(grad)
    return grad_list

def get_dist_togoal(i, X, Y):
    x_i = X[i]
    y_i = Y[i]
    x_j = X[j]
    y_j = Y[j]

    return math.sqrt((x_i-x_j)**2 + (y_i-y_j)**2)

def get_dist_tuples(T1, T2):
    dim = len(T1)
    sqare_sum = 0
    for i in range(dim):
        sqare_sum = sqare_sum + (T1[i] - T2[i])**2
    return math.sqrt(sqare_sum)


def get_traj_g_et(P_r, g_et_pcl, res):
    X_pcl = g_et_pcl[0]
    Y_pcl = g_et_pcl[1]
    Z_pcl = g_et_pcl[2]

    i_r = get_robot_idx(P_r, X_pcl, Y_pcl)

    traj_idx_list = [i_r]

    while 1:
        i_adj_list = get_adj_idxs(i_r, X_pcl, Y_pcl, res*1.05)
        grad_list = get_gradients(i_r, i_adj_list, X_pcl, Y_pcl, Z_pcl)
        if min(grad_list) < 0:
            i_next = i_adj_list[np.argmin(grad_list)]
            traj_idx_list.append(i_next)
            i_r = i_next
        else:
            break

    X_t = [X_pcl[i] for i in traj_idx_list]
    Y_t = [Y_pcl[i] for i in traj_idx_list]
    Z_t = [Z_pcl[i] for i in traj_idx_list]

    print('got trajectory')
    return [X_t, Y_t, Z_t]

def get_traj_est(P_r, NN, g_et_pcl, res, d_local):
    # NN_est_sp : shortest paths of NN est
    # NN_est_c : costs of NN est

    traj_NN_node, traj_NN_coord = get_traj_NN_est(NN)

    # entry trip
    traj_entry = [[P_r[0], traj_NN_coord[0][0]],
                  [P_r[1], traj_NN_coord[0][1]],
                  [P_r[2], 0.0]]

    # middle NN trip
    traj_middle = [[coord[0] for coord in traj_NN_coord],
                   [coord[1] for coord in traj_NN_coord],
                   [0 for coord in traj_NN_coord]]

    # terminal frontier exploration
    P_t = (traj_NN_coord[-1][0], traj_NN_coord[-1][1], 0)
    # X_l, Y_l, Z_l = get_local_pcl(P_t, g_et_pcl[0], g_et_pcl[1], g_et_pcl[2], d_local)
    X_l, Y_l, Z_l = [ g_et_pcl[0], g_et_pcl[1], g_et_pcl[2]]
    traj_terminal = get_traj_g_et(P_t, [X_l, Y_l, Z_l], res)

    X_t = traj_entry[0] + traj_middle[0] + traj_terminal[0]
    Y_t = traj_entry[1] + traj_middle[1] + traj_terminal[1]
    Z_t = traj_entry[2] + traj_middle[2] + traj_terminal[2]

    return [X_t, Y_t, Z_t], [X_l, Y_l, Z_l], P_t

def get_traj_NN_est(NN):
    node_isrobot = [node for node, value in nx.get_node_attributes(NN, 'is_robot').items() if value == True][0]
    to_go_dict = nx.get_node_attributes(NN, 'to_go')
    cost_dict = nx.get_node_attributes(NN,'cost')
    pos_dict = nx.get_node_attributes(NN, 'pos')

    min_cost = 10000000
    for node, cost in cost_dict.items():
        if cost < min_cost and to_go_dict[node]==True:
            min_cost = cost
            node_min_cost = node

    sp = nx.shortest_path(NN, source=node_isrobot, target=node_min_cost, weight='dist')

    traj_NN_node = sp
    traj_NN_coord = [pos_dict[node] for node in sp]

    return traj_NN_node, traj_NN_coord

def get_NN_est(NN, beta):
    #
    # cost = sp_len + beta*(1-value)
    n_r = [i for i, v in nx.get_node_attributes(NN, 'is_robot').items() if v == True][0]
    # sp = nx.shortest_path(NN, source=n_r, weight='dist')
    sp_len = nx.shortest_path_length(NN, source=n_r, weight='dist')
    value = nx.get_node_attributes(NN, 'value')

    cost = {}
    for (k, v_sp), (_, v_v) in zip(sp_len.items(), value.items()):
        cost[k] = v_sp + beta * (1.0 - v_v)
        NN.nodes()[k]['cost'] = v_sp + beta * (1.0 - v_v)

    return NN

def get_local_pcl(O, X, Y, Z, d):
    # O : robot odom : [x,y,z]
    # X, Y, Z map pcl
    x_r = O[0]
    y_r = O[1]
    # i_close_list = [i for i, (x, y) in enumerate(zip(X, Y)) if abs(x_r-x)<d and abs(y_r-y)<d]
    i_close_list = [i for i, (x, y) in enumerate(zip(X, Y)) if math.sqrt((x_r-x)**2+(y_r-y)**2) < d]
    X_close = [v for i, v in enumerate(X) if i in i_close_list]
    Y_close = [v for i, v in enumerate(Y) if i in i_close_list]
    Z_close = [v for i, v in enumerate(Z) if i in i_close_list]

    return X_close, Y_close, Z_close

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

def get_distance_trans_array(map, idx_target):
    queue_i = [idx_target[0]]
    queue_j = [idx_target[1]]
    dist_trans = np.ones(np.shape(map))*100000.0
    dist_trans[idx_target[0], idx_target[1]] = 0.0
    while len(queue_i) > 0:
        i = queue_i.pop(0)
        j = queue_j.pop(0)
        adj_s = get_straight_idxs((i,j))
        adj_d = get_diagonal_idxs((i,j))
        for k in range(4):
            if map[adj_s[k][0], adj_s[k][1]]==0 and (dist_trans[i,j] + 1.0 < dist_trans[adj_s[k][0], adj_s[k][1]]):
                dist_trans[adj_s[k][0], adj_s[k][1]] = dist_trans[i,j] + 1.0
                queue_i.append(adj_s[k][0])
                queue_j.append(adj_s[k][1])
            if map[adj_d[k][0], adj_d[k][1]]==0 and (dist_trans[i,j] + 1.414 < dist_trans[adj_d[k][0], adj_d[k][1]]):
                dist_trans[adj_d[k][0], adj_d[k][1]] = dist_trans[i,j] + 1.414
                queue_i.append(adj_d[k][0])
                queue_j.append(adj_d[k][1])
    idx_dummy = np.where(dist_trans==100000.0)
    dist_trans[idx_dummy[0], idx_dummy[1]] = 0
    return dist_trans

def c_danger(obs_trans_array, c, d_opt, res):
    d_opt_cell = d_opt/res # from meter to cell
    if obs_trans_array[c[0], c[1]] <= d_opt_cell:
        return (d_opt_cell - obs_trans_array[c[0],c[1]])**1.5
    else:
        return (obs_trans_array[c[0],c[1]] - d_opt_cell)**1.5

def get_path_trans_array(map, obs_trans_array, idx_target, alpha, d_opt, res):
    # dist_trans_array = get_distance_trans_array(map, idx_target)
    queue_i = [idx_target[0]]
    queue_j = [idx_target[1]]
    path_trans = np.ones(np.shape(map))*100000.0
    path_trans[idx_target[0], idx_target[1]] = 0.0
    while len(queue_i) > 0:
        i = queue_i.pop(0)
        j = queue_j.pop(0)
        adj_s = get_straight_idxs((i,j))
        adj_d = get_diagonal_idxs((i,j))
        for k in range(4):
            if map[adj_s[k][0], adj_s[k][1]]==0 and (path_trans[i,j] + 1 + alpha*(c_danger(obs_trans_array,(adj_s[k][0],adj_s[k][1]),d_opt,res)) < path_trans[adj_s[k][0], adj_s[k][1]]):
                path_trans[adj_s[k][0], adj_s[k][1]] = path_trans[i,j] + 1 + alpha*(c_danger(obs_trans_array,(adj_s[k][0],adj_s[k][1]),d_opt,res))
                queue_i.append(adj_s[k][0])
                queue_j.append(adj_s[k][1])
            if map[adj_d[k][0], adj_d[k][1]]==0 and (path_trans[i,j] + 1.414 + alpha*(c_danger(obs_trans_array,(adj_d[k][0],adj_d[k][1]),d_opt,res)) < path_trans[adj_d[k][0], adj_d[k][1]]):
                path_trans[adj_d[k][0], adj_d[k][1]] = path_trans[i,j] + 1.414 + alpha*(c_danger(obs_trans_array,(adj_d[k][0],adj_d[k][1]),d_opt,res))
                queue_i.append(adj_d[k][0])
                queue_j.append(adj_d[k][1])
    idx_dummy = np.where(path_trans==100000.0)
    path_trans[idx_dummy[0], idx_dummy[1]] = 0
    return path_trans

def get_exploration_transform_array(map, frontier_trans_array, obs_trans_array, alpha, d_opt, res):
    idx_frontier = np.where(frontier_trans_array==1)
    queue_i = list(idx_frontier[0])
    queue_j = list(idx_frontier[1])
    exp_trans = np.ones(np.shape(map))*100000.0
    exp_trans[idx_frontier[0], idx_frontier[1]] = 0.0
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
    idx_dummy = np.where(exp_trans==100000.0)
    exp_trans[idx_dummy[0], idx_dummy[1]] = 0
    return exp_trans

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

def get_frontier_value_trans_array(node_pos_dict, node_value_dict, map, frontier_trans_array):
    # get node pos in array
    # node_pos_dict = get_nx_node_pos_array(G, res, origin)
    # node_value_dict = nx.get_node_attributes(G, 'value')
    idx_nodes = node_pos_dict.values()
    value_nodes = node_value_dict.values()
    idx_frontiers = np.where(frontier_trans_array == 1)
    queue_i = [idx[0] for idx in idx_nodes]
    queue_j = [idx[1] for idx in idx_nodes]
    queue_v = [value for value in value_nodes]
    queue_f = [idx for idx in zip(list(idx_frontiers[0]), list(idx_frontiers[1]))]

    frontier_value_trans = np.ones(np.shape(map))*(-1)
    value_trans = np.ones(np.shape(map))*(-1)
    for iter, (i,j) in enumerate(zip(queue_i, queue_j)):
        value_trans[i, j] = queue_v[iter]
    while len(queue_i)>0 and len(queue_f)>0:
        i = queue_i.pop(0)
        j = queue_j.pop(0)
        v = queue_v.pop(0)

        adj_s = get_straight_idxs((i,j))
        adj_d = get_diagonal_idxs((i,j))

        for k in range(4):
            try:
                if value_trans[adj_s[k][0], adj_s[k][1]] == -1 and map[adj_s[k][0], adj_s[k][1]] == 0:
                    value_trans[adj_s[k][0], adj_s[k][1]] = v
                    queue_i.append(adj_s[k][0])
                    queue_j.append(adj_s[k][1])
                    queue_v.append(v)
                    if adj_s[k] in queue_f:
                        frontier_value_trans[adj_s[k][0],adj_s[k][1]] = v
                        queue_f.remove(adj_s)
                if value_trans[adj_d[k][0], adj_d[k][1]] == -1 and map[adj_s[k][0], adj_s[k][1]] == 0:
                    value_trans[adj_d[k][0], adj_d[k][1]] = v
                    queue_i.append(adj_d[k][0])
                    queue_j.append(adj_d[k][1])
                    queue_v.append(v)
                    if adj_d[k] in queue_f:
                        frontier_value_trans[adj_d[k][0],adj_d[k][1]] = v
                        queue_f.remove(adj_d)
            except:
                pass
    return frontier_value_trans


def convert_cart2array(pos_cart, res, map_origin):
    # position of point in cartesian coordinate
    # res : map resolution
    # o : map origin
    x_cart = pos_cart[0]
    y_cart = pos_cart[1]
    i_array = int((y_cart - map_origin[1])/res)
    j_array = int((x_cart - map_origin[0])/res)
    return (i_array, j_array)


def convert_cart2plot(pos_cart, res, map_origin):
    # position of point in cartesian coordinate
    # res : map resolution
    # o : map origin
    x_cart = pos_cart[0]
    y_cart = pos_cart[1]
    i_array = int((x_cart - map_origin[0])/res)
    j_array = int((y_cart - map_origin[1])/res)
    return (i_array, j_array)

def get_nx_node_pos_array(G, res, map_origin):
    pos_cart_dict = nx.get_node_attributes(G, 'pos')
    pos_cart_array = {}
    for node , pos_cart in pos_cart_dict.items():
        pos_cart_array[node] = convert_cart2array(pos_cart, res, map_origin)
    return pos_cart_array

def get_nx_node_pos_plot(G, res, map_origin):
    pos_cart_dict = nx.get_node_attributes(G, 'pos')
    pos_cart_array = {}
    for node , pos_cart in pos_cart_dict.items():
        pos_cart_array[node] = convert_cart2plot(pos_cart, res, map_origin)
    return pos_cart_array

def gcd(a, b):
    if b > a:
        tmp = a
        a = b
        b = tmp
    while b > 0:
        c = b
        b = a % b
        a = c
    return a


def lcm(a, b):
    return a * b // gcd(a, b)