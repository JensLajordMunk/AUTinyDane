import numpy as np

class State:
    def __init__(self,config):
        self.config = config
        self.leg_pair_in_swing = np.array([False, False])
        self.firstIt = True
        self.legpair_phases_remaining = np.array([0,0])
        self.foot_locations = np.zeros((3, 4))
        self.velocityX = 0#self.config.max_velocityX
        self.velocityY = 0#self.config.max_velocityY

        self.trot_yaw = 0
        self.stance_yaw_pair = np.array([0.0,0.0])
        self.stand_yaw = 0
        self.stand_pitch = 0
        self.stand_roll = 0

        #self.joint_angles = np.array([[  0.,  0.,  0.,  0.],
                                 #[ 45., 45., 45., 45.],
                                 #[ -45., -45., -45., -45.]])
