
class map:
    def __init__(self):
        self.map = []
        self.res = []
        self.origin = []
        self.map_msg = []

    def __getitem__(self, key):
        return getattr(self, key)

    def __setitem__(self, key, value):
        if hasattr(self, key):
            return setattr(self, key, value)
        else:
            print('not defined variable')

class hector:
    def __init__(self):
        self.g_et_pcl = [[], [], []] # grid cell exploration_transform pointclouds list
        self.g_ot_pcl = [[], [], []]
        self.g_et_pcl_local = [[], [], []] # local grid cell exploration_transform pointclouds list
        self.f_pcl_local = [[], [], []]  # frontier pointclouds of local grid cell et list
        self.path = [[], [], []]  # path from hector


    def __getitem__(self, key):
        return getattr(self, key)

    def __setitem__(self, key, value):
        if hasattr(self, key):
            return setattr(self, key, value)
        else:
            print('not defined variable')

class navigation_network:
    def __init__(self):
        self.NN = [] # navigation network

    def __getitem__(self, key):
        return getattr(self, key)

    def __setitem__(self, key, value):
        if hasattr(self, key):
            return setattr(self, key, value)
        else:
            print('not defined variable')

class plan:
    def __init__(self):
        self.path = [[], [], []] # exploration_transform pointclouds list

    def __getitem__(self, key):
        return getattr(self, key)

    def __setitem__(self, key, value):
        if hasattr(self, key):
            return setattr(self, key, value)
        else:
            print('not defined variable')

class robot:
    def __init__(self):
        self.pos = [] # robot position [x,y,z]

    def __getitem__(self, key):
        return getattr(self, key)

    def __setitem__(self, key, value):
        if hasattr(self, key):
            return setattr(self, key, value)
        else:
            print('not defined variable')

class time_stamp:
    def __init__(self):
        self.now = 0  # simulation time
        self.map = 0 # map topic
        self.et = 0   # exploration transform from hector
        self.path_hector = 0 # path from hector
        self.odom = 0 # robot odom

    def __getitem__(self, key):
        return getattr(self, key)

    def __setitem__(self, key, value):
        if hasattr(self, key):
            return setattr(self, key, value)
        else:
            print('not defined variable')

class flag:
    def __init__(self):
        self.p_map = False
        self.p_et = False # processing of exploration transform
        self.p_ot = False

    def __getitem__(self, key):
        return getattr(self, key)

    def __setitem__(self, key, value):
        if hasattr(self, key):
            return setattr(self, key, value)
        else:
            print('not defined variable')