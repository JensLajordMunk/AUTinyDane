import numpy as np
class ServoCalibration:
    def __init__(self):
        self.MICROS_PER_RAD = 180/np.pi*np.array(
                                [[ 12.483,  0.,  0.,  0.],
                                 [ 10.234364777777687, 0., 0., 0.],
                                 [ 11.86187, 0., 0., 0.]]
                                )# TODO: Insert values from servo motor: micros_per_rad = dPWM/dAngle
        self.NEUTRAL_ANGLE_DEGREES = np.array(
                                [[  0.,  0.,  0.,  0.],
                                 [ 45., 45., 45., 45.],
                                 [ -45., -45., -45., -45.]]
                                )
        self.neutral_position_pwm = np.array(
                                [[ 1700.64,  0.,  0.,  0.],
                                 [ 1563.4196590497777, 0., 0., 0.],
                                 [ 1738.4958093999999, 0., 0., 0.]]
                                )