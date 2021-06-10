class Node_ENN:
    def __init__(self, index, x, y, area, level, visibility, node_type, to_go, isrobot):
        self.index = index
        self.x = x
        self.y = y
        self.area = area
        self.level = level
        self.visibility = visibility
        self.node_type = node_type
        self.to_go = to_go
        self.isrobot = isrobot
        self.category_str = ''
        self.category_idx = -1

class Node_voronoi:
    def __init__(self, index, x, y, to_go, isrobot):
        self.index = index
        self.x = x
        self.y = y
        self.to_go = to_go
        self.isrobot = isrobot
        self.category_str = ''
        self.category_idx = -1

class flag_state:
    def __init__(self):
        self.init_NN = False # navigation network
        self.use_map = False
        self.publish_floorplan = False

    def __getitem__(self,key):
        return getattr(self, key)

    def __setitem__(self,key,value):
        if hasattr(self, key):
            return setattr(self, key, value)
        else:
            print('not defined variable')

class time_stamp:
    def __init__(self):
        self.now  = 0  # time now
        self.status = 0 # robot mission status
        self.map  = 0 # map data
        self.odom = 0 # robot odom
        self.cmd  = 0 # goal command
        self.stop = 0 # last time robot stopped
        self.move = 0 # last time robot moved
        self.graph = 0 # only for voronoi graph

    def __getitem__(self, key):
        return getattr(self, key)

    def __setitem__(self, key, value):
        if hasattr(self, key):
            return setattr(self, key, value)
        else:
            print('not defined variable')

class robot_state:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.pos = (self.x, self.y)

        self.x_pix = 0
        self.y_pix = 0
        self.pos_pix = (self.x_pix, self.y_pix)

        self.x_pix_scaled = 0
        self.y_pix_scaled = 0
        self.pos_pix_scaled = (self.x_pix_scaled, self.y_pix_scaled)

        self.x_pix_local_scaled = 0
        self.y_pix_local_scaled = 0

    def __getitem__(self, key):
        return getattr(self, key)

    def __setitem__(self, key, value):
        if hasattr(self, key):
            return setattr(self, key, value)
        else:
            print('not defined variable')

class map_state:
    def __init__(self):
        self.map_1d         = [] # (unknown:255, free:0, occ:100)
        self.map_2d         = [] # (unknown:255, free:0, occ:100)
        self.map_2d_1max = [] # (occ:0, unknown:0.5, free:1)
        self.map_2d_1max_scaled = []
        self.map_2d_local   = [] # (unknown:255, free:0, occ:100)
        self.map_2d_local_1max = [] # (occ:0, unknown:0.5, free:1)
        self.map_2d_local_1max_scaled = []
        self.vis_map = []
        self.class_map = []
        self.res            = 0
        self.origin = []
        self.len_x_pix      = 0
        self.len_y_pix      = 0
        self.len_x          = 0
        self.len_y          = 0
        self.area_per_pixel = 0
        self.origin_x       = 0
        self.origin_y       = 0
        self.vertices = []

        self.index_map = []
        self.centroids = []

        self.local_network = []
        self.global_network = []

        self.img_floorplan = []

        self.map_msg = []

    def __getitem__(self, key):
        return getattr(self, key)

    def __setitem__(self, key, value):
        if hasattr(self, key):
            return setattr(self, key, value)
        else:
            print('not defined variable')

class mission_state:
    def __init__(self):
        self.goal_x = 0
        self.goal_y = 0
        self.status = 0

    def __getitem__(self, key):
        return getattr(self, key)

    def __setitem__(self, key, value):
        if hasattr(self, key):
            return setattr(self, key, value)
        else:
            print('not defined variable')