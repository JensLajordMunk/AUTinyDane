import numpy as np

class State:
    def __init__(self):
        self.foot_locations = np.zeros((3, 4))
        self.joint_angles = np.array([[  0.,  0.,  0.,  0.],
                                 [ 45., 45., 45., 45.],
                                 [ -45., -45., -45., -45.]])
