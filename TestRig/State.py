import numpy as np

class State:
    def __init__(self,config):
        self.config = config
        self.leg_pair_in_swing = np.array([False, False])
        self.legpair_start_time = [0.0, 0.0]
        self.firstIt = True
        #self.legpair_phases_remaining = np.array([0,0]) # Legacy code from GaitPlanner.py, we are currently using V2
        self.foot_locations = np.zeros((3, 4))
        self.velocityX = 0.0
        self.velocityY = 0.0

        self.trot_yaw = 0
        self.stance_yaw_pair = np.array([0.0, 0.0])
        self.stance_roll_pair = np.array([0.0, 0.0])
        self.stance_pitch_pair = np.array([0.0, 0.0])
        self.stand_yaw = 0
        self.stand_pitch = 0
        self.stand_roll = 0

        self.stand_x = 0
        self.stand_y = 0
        self.stand_z = 0

        #self.joint_angles = np.array([[  0.,  0.,  0.,  0.],
                                 #[ 45., 45., 45., 45.],
                                 #[ -45., -45., -45., -45.]])
