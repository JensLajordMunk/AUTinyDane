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
                                [[ 1, 1, 1, 1],
                                 [ 1, -1, 1, 1],
                                 [ 1, -1, 1, 1]]
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
        self.body_width = 0.1# TODO: Insert width of body
        self.body_length = 0.2# TODO: Insert length of body
        self.body_height = 0.1131# TODO: Insert height to leg origin at neutral stance

        #-------------- Gait parameters ----------------
        self.arcR = 0.012
        self.velocity = 0.1
        self.stancetime = 1
        self.swingtime = self.stancetime/3 # semi trot gait pattern
        self.step_height = 0.06
        self.frequency = 125
        self.Z_zero = 0.1131 # The height of the leg
        self.leg_pairs = np.array([[0,2],
                                  [1,3]])
        self.leg_pair_in_swing = np.array([False, False])
        self.legpair_phases_remaining = np.array([0,0])
        self.firstIt = True



