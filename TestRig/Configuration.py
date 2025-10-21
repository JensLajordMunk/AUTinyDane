import numpy as np
from ServoCalibration import ServoCalibration

class PWMParams:
    def __init__(self):
        self.range = 65535
        self.freq = 330


class ServoParams:
    def __init__(self):

        servo_calibration = ServoCalibration()

        self.neutral_position_pwm = servo_calibration.neutral_position_pwm # TODO: Remember to change in ServoCalibration.py
        self.micros_per_rad = servo_calibration.MICROS_PER_RAD  # TODO: Remember to change in ServoCalibration.py

        # The neutral angle of the joint relative to the modeled zero-angle in degrees, for each joint
        self.neutral_angle_degrees = servo_calibration.NEUTRAL_ANGLE_DEGREES

        self.servo_multipliers = np.array(
                                [[ 1, -1, 1, -1],
                                 [ 1, 1, 1, 1],
                                 [ 1, 1, 1, 1]]
                                )# TODO: Find array of sign changes for servos

    @property # The property decorator makes sure that if neutral_angle_degrees is updated then neutral_angle updates
    def neutral_angles(self):
        return self.neutral_angle_degrees * np.pi / 180.0  # Convert to radians

class RobotConfig:
    def __init__(self):
        #-------------- Geometry ----------------
        self.abduction_offset =0.04 # TODO: Insert abduction offset
        self.abduction_offsets = np.array([
            self.abduction_offset,
            -self.abduction_offset,
            self.abduction_offset,
            -self.abduction_offset
        ])
        self.leg_up =0.08# TODO: Insert length of upper leg
        self.leg_low =0.08# TODO: Insert length of lower leg
