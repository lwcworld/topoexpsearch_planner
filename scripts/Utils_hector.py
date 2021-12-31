import networkx as nx
import math
import matplotlib.pyplot as plt
import copy
import numpy as np

def get_frontier_zerovalue_trans_array(local_map, frontier_trans_array):
    idx_frontiers = np.where(frontier_trans_array==1)
    frontier_value_trans = np.ones(np.shape(local_map))*(-1)
    frontier_value_trans[idx_frontiers[0], idx_frontiers[1]] = 0
    return frontier_value_trans

def convert_path_array2cart(path_array, res, origin_array, start_cart):
    path_cart = []
    for point_array in path_array:
        i_array = point_array[0]
        j_array = point_array[1]
        # x_cart = (i_array - origin_array[0]) * res + start_cart[0]
        # y_cart = (j_array - origin_array[1]) * res + start_cart[1]

        x_cart = (j_array - origin_array[1]) * res + start_cart[0]
        y_cart = (i_array - origin_array[0]) * res + start_cart[1]
        point_cart = (x_cart, y_cart)
        path_cart.append(point_cart)
    return path_cart