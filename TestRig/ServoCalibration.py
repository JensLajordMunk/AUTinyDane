import numpy as np
class ServoCalibration:
    def __init__(self):
        self.MICROS_PER_RAD = 180/np.pi*np.array(
                                [[ 11.928,  10.688,  11.864,  10.918],
                                 [ 11.54, 11.2, 10.811, 11.395],
                                 [ 11.994, 11.755, 11.397, 11.932]]
                                )# TODO: Insert values from servo motor: micros_per_rad = dPWM/dAngle
        self.NEUTRAL_ANGLE_DEGREES = np.array(
                                [[ -6.0, -34.0, -48., -8.],
                                 [ 45., 45.-5., 45.-14., 45.+3.],
                                 [ -45.+10., -45.+22., -45.+12., -45.+27.]]
                                )
        self.neutral_position_pwm = np.array(
                                [[ 1625.7,  1726.6,  1588.108,  1522.604],
                                 [ 1619.7, 1845.3, 1720.77, 1657.5205],
                                 [ 1631.7, 1733.3, 1660.11, 1588.904]]
                                )