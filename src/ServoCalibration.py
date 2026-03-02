import numpy as np

class ServoCalibration:
    def __init__(self):
        self.MICROS_PER_RAD = 180/np.pi*np.array(
                                [[ 11.928,  10.688,  11.864,  10.918],
                                 [ 11.412, 12.880, 12.613, 12.914],
                                 [ 11.727, 12.669, 12.347, 12.860]]
                                )
        self.NEUTRAL_ANGLE_DEGREES = np.array(
                                [[ 0, 0, 0, 0.],
                                 [ -48, -48, -48, -48],
                                 [ 0, 0, 0, 0]]
                                )
        self.neutral_position_pwm = np.array(
                                [[ 1625.7,  1726.6,  1588.108,  1522.604],
                                 [ 1585.1, 1738.9, 1688.3, 1737.3],
                                 [ 1607.7, 1745.1, 1711.4, 1720.2]]
                                )