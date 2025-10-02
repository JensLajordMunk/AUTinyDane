import numpy as np
from ServoCalibration import NEUTRAL_ANGLE_DEGREES, MICROS_PER_RAD

class PWMParams:
    def __init__(self):
        self.pins = # Insert np array of pins used for each pin
        self.range = 65535
        self.freq = 330


class ServoParams:
    def __init__(self):
        self.neutral_position_pwm = 1520
        self.micros_per_rad = MICROS_PER_RAD  # Remember to change in ServoCalibration.py

        # The neutral angle of the joint relative to the modeled zero-angle in degrees, for each joint
        self.neutral_angle_degrees = NEUTRAL_ANGLE_DEGREES # Remember to change in ServoCalibration.py

        self.servo_multipliers = # Find array of sign changes for servos

    @property # The property decorator makes sure that if neutral_angle_degrees is updated then neutral_angle updates
    def neutral_angles(self):
        return self.neutral_angle_degrees * np.pi / 180.0  # Convert to radians


class RobotConfig:
    def __init__(self):
        #-------------- Geometry ----------------
        self.abduction_offset = # Insert abduction offset
        self.abduction_offsets = np.array([
            -self.abduction_offset,
            self.abduction_offset,
            -self.abduction_offset,
            self.abduction_offset
        ])
        self.leg_up = # Insert length of upper leg
        self.leg_low = # Insert length of lower leg

        self.dt = 0.1 # Tick time
        self.degree_pr_second = 5