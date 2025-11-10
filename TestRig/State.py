import numpy as np

class State:
    def __init__(self,config):
        self.config = config
        self.leg_pair_in_swing = np.array([False, False])
        self.firstIt = True
        self.legpair_phases_remaining = np.array([0,0])
        self.foot_locations = np.zeros((3, 4))
        self.velocity = self.config.max_velocity
        #self.joint_angles = np.array([[  0.,  0.,  0.,  0.],
                                 #[ 45., 45., 45., 45.],
                                 #[ -45., -45., -45., -45.]])
