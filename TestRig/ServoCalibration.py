import numpy as np
class ServoCalibration:
    def __init__(self):
        self.MICROS_PER_RAD = None# TODO: Insert values from servo motor: micros_per_rad = dPWM/dAngle
        self.NEUTRAL_ANGLE_DEGREES = np.array(
                                [[  0.,  0.,  0.,  0.],
                                 [ 45., 45., 45., 45.],
                                 [-45.,-45.,-45.,-45.]]
                                )