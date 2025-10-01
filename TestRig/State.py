import numpy as np

class State:
    def __init__(self):
        self.foot_locations = np.zeros((3, 4))
        self.joint_angles = np.zeros((3, 4))
