import numpy as np
class ServoCalibration:
    def __init__(self):
        self.MICROS_PER_RAD = 180/np.pi*np.array(
                                [[ 12.483,  0.,  0.,  0.],
                                 [ 11.650799999999897, 0, 0, 0],
                                 [ 11.914824999999977, 0, 0, 0]]
                                )# TODO: Insert values from servo motor: micros_per_rad = dPWM/dAngle
        self.NEUTRAL_ANGLE_DEGREES = np.array(
                                [[  0.,  0.,  0.,  0.],
                                 [ 0, 0, 0, 0],
                                 [ 0, 0, 0, 0]]
                                )
        self.neutral_position_pwm = np.array(
                                [[ 1700.64,  0.,  0.,  0.],
                                 [ 1699.6369290497764, 0, 0, 0],
                                 [ 1699.9977043769331, 0, 0, 0]]
                                )