import numpy as np
from ServoCalibration import ServoCalibration


class PWMParams:
    def __init__(self):
        self.range = 65535
        self.freq = 200


class ServoParams:
    def __init__(self):

        servo_calibration = ServoCalibration()

        self.neutral_position_pwm = (
            servo_calibration.neutral_position_pwm
        )
        self.micros_per_rad = (
            servo_calibration.MICROS_PER_RAD
        )

        # The neutral angle of the joint relative to the modeled zero-angle in degrees, for each joint
        self.neutral_angle_degrees = servo_calibration.NEUTRAL_ANGLE_DEGREES

        self.servo_multipliers = np.array(
            [[1, 1, 1, 1],
             [1, -1, 1, -1],
             [1, -1, 1, -1]]
        )

    @property  # The property decorator makes sure that if neutral_angle_degrees is updated then neutral_angle updates
    def neutral_angles(self):
        return self.neutral_angle_degrees * np.pi / 180.0  # Convert to radians


class RobotConfig:
    def __init__(self):
        # -------------- Geometry ----------------
        self.abduction_offset = 0.04235
        self.abduction_offsets = np.array(
            [
                self.abduction_offset,
                -self.abduction_offset,
                self.abduction_offset,
                -self.abduction_offset,
            ]
        )

        self.leg_up = 0.08
        self.leg_low = 0.09278
        self.body_width = 0.09012
        self.body_length = 0.16993
        self.body_height = 0.13

        # -------------- Gait parameters ----------------
        self.swing_method = "TRIANGULAR"
        self.arcR = 0.012
        self.velocity = 0.02
        self.stancetime = 0.4
        self.swingtime = self.stancetime / 3  # semi trot gait pattern
        self.step_height = 0.04
        self.frequency = 30
        self.leg_pairs = np.array([[0, 3], [1, 2]])



        # -------------- Max parameters ----------------
        self.max_velocityX = 0.15
        self.max_velocityY = 0.02
        self.max_acceleration = 0.05 #m/s^2
        self.max_yaw_acceleration = 10 # deg/s^2
        self.max_yaw_rate =  20 #deg/s
        self.max_yaw_stand = 20
        self.max_pitch = 20
        self.max_roll = 20
        self.max_yaw_rate =  20 #deg/s
        self.max_yaw_stand = 20 #deg
        self.max_pitch = 20 #deg
        self.max_roll = 20 #deg
        self.max_x = 0.04 #meters
        self.max_y = 0.04 #meters
        self.max_z = 0.04 #meters

        # -------------- PD controller ----------------
        self.k_p_stand_pitch = 0.01
        self.k_d_stand_pitch = 0.01
        self.k_p_stand_roll = 0.01
        self.k_d_stand_roll = 0.01
        self.k_p_stance_pitch = 0.0
        self.k_d_stance_pitch = 0.0
        self.k_p_stance_roll = 0.0
        self.k_d_stance_roll = 0.0
