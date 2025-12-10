import numpy as np
from src.ServoCalibration import ServoCalibration


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
             [-1, 1, -1, 1]]
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
        self.leg_low = 0.09225 # Should be measured based on neutral position
        self.body_width = 0.09012
        self.body_length = 0.16993
        self.body_height = 0.13
        self.l1 = 0.024
        self.l2 = 0.03
        self.l3 = 0.0277
        self.l4 = 0.1015
        self.l5 = 0.0377
        self.theta_low = 155.45*np.pi/180.0 #Should be measured based on neutral position
        self.theta_pizza = 82.38*np.pi/180.0
        self.servo_offset_x = 0.022
        self.servo_offset_z = 0.020

        # -------------- Gait parameters ----------------
        self.swing_method = "CHEETAH"
        self.stance_method = "BEZIER"
        self.arcR = 0.012
        self.stancetime = 0.5
        self.swingtime = self.stancetime / 3  # semi trot gait pattern
        self.step_height = 0.04
        self.frequency = 30
        self.leg_pairs = np.array([[0, 3], [1, 2]])

        # -------------- Max parameters ----------------
        self.max_acceleration = 0.07 #m/s^2
        self.max_yaw_acceleration = 40 # deg/s^2
        self.max_yaw_rate =  50 #deg/s
        self.max_yaw_stand = 20 #deg
        self.max_pitch = 20 #deg
        self.max_roll = 20 #deg
        self.max_x = 0.07 #meters
        self.max_y = 0.06 #meters
        self.max_z = 0.04 #meters
        self.max_velocityX = min(0.3, (self.max_x-self.arcR)*2/self.stancetime) #m/s
        self.max_velocityY = min(0.1, (self.max_y-self.arcR)*2/self.stancetime) #m/s
