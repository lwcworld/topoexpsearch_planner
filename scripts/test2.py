import numpy as np
from Utils_GEST import *
from Params import *


map = np.load('test_map2.npy')
p = param()


frontier_trans_array = get_frontier_trans_array(map, p['min_dist_frontier_obs'], 0.05)

m = np.zeros(np.shape(map))
idx_free = np.where(map==0)
idx_occ = np.where(map==100)
idx_unknown = np.where(map==-1)
idx_frontier = np.where(frontier_trans_array==1)
m[idx_free] = 1
m[idx_occ] = 2
m[idx_unknown] = 3
m[idx_frontier] = 5
plt.imshow(m)
plt.gca().invert_yaxis()
plt.show()
plt.imshow(frontier_trans_array)
plt.gca().invert_yaxis()
plt.show()