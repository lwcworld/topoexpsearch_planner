import numpy as np

class param:
    def __init__(self):
        self.sim_speed = 2.

        # frequency in simulation time
        self.freq_est  = 1.
        self.freq_ctrl = 1.
        self.freq_rviz = 1.

        # radius
        self.r            = 7.0 # [m]
        self.r_pix_scaled = 0

        #
        self.def_occ     = 0.0
        self.def_unknown = 0.5
        self.def_free    = 1.0

        #
        self.scale_per   = 10.0

        #
        self.class_level = 5

        self.remove_small_area    = True
        self.small_area_threshold = 4.0
        self.gate                 = 5.0

        # topo algorithm (ENN, voronoi)
        self.topo_alg = 'voronoi'

        #
        self.local_map_width_height = 300

        # category colomap
        # self.cmap_category = {'lab': [(0, 0, 255)],
        #                      'corridor': [(0, 150, 255), (255, 255, 0), (255,255,255)],
        #                      'share': [(0, 255, 255)],
        #                      'storage': [(0, 255, 0)],
        #                      'toilet': [(255, 0, 0)],
        #                      'maintenence': [(255, 0, 150)],
        #                      'food': [(255, 0, 255)]}

        self.cmap_category = {'lab': [(0, 0, 255)],
                             'corridor': [(0, 150, 255), (255, 255, 0)],
                             'share': [(0, 255, 255)],
                             'storage': [(0, 255, 0)],
                             'toilet': [(255, 0, 0)],
                             'maintenence': [(0, 255, 0)],
                             'food': [(0, 255, 255)],
                             'pass': [(255,255,255)]}

        # self.imap_category = {'lab':0, 'corridor':1, 'share':2, 'storage':3, 'toilet':4, 'maintenence':5, 'food':6}
        self.imap_category = {'lab': 1, 'corridor': 2, 'share': 3, 'storage': 4, 'toilet': 5, 'maintenence': 4, 'food': 3, 'pass':0}

        # directories
        self.dir_log = '../dataset/log/'

        # EST related
        self.min_dist_frontier_obs = 0.2
        self.alpha_G = 0.05  # obstacle weight
        self.beta_G = 1.0 # weight for object search probability (OSP)

        self.alpha = 1.0  # obstacle weight
        self.beta = 3000.0 # weight for object search probability (OSP)
        self.d_opt = 1.0


    def __getitem__(self, key):
        return getattr(self, key)

    def __setitem__(self, key, value):
        if hasattr(self, key):
            return setattr(self, key, value)
        else:
            print('not defined variable')