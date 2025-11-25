import numpy as np
import board
import busio
from adafruit_pca9685 import PCA9685
from mpu6050 import mpu6050
from Configuration import PWMParams, ServoParams
# The following line is a fix for raspberrypi4
import types

 
class HardwareInterface:
    def __init__(self):
        self.pwm_params = PWMParams()
        self.servo_params = ServoParams()

        # The following line is a fix for raspberrypi4 // GPIO PINS 3 AND 2
        self.board = types.SimpleNamespace(SCL=3, SDA=2)
        self.i2c = busio.I2C(self.board.SCL, self.board.SDA)
        self.pca0 = PCA9685(self.i2c, address=0x40)
        self.pca0.frequency = self.pwm_params.freq
        self.pca1 = PCA9685(self.i2c, address=0x41)
        self.pca1.frequency = self.pwm_params.freq
        self.pca2 = PCA9685(self.i2c, address=0x42)
        self.pca2.frequency = self.pwm_params.freq
        self.pca3 = PCA9685(self.i2c, address=0x43)
        self.pca3.frequency = self.pwm_params.freq

        self.channels = [0, 1, 2]

        self.mpu = mpu6050(0x68)

    def set_actuator_positions(self, joint_angles):
        for leg_index in range(4):
            for motor_index in range(3):
                joint_angle = joint_angles[motor_index,leg_index]
                self.servo_command(leg_index, motor_index, joint_angle)

    def set_actuator_position(self, joint_angle, leg_index, motor_index):
        self.servo_command(leg_index, motor_index, joint_angle)

    def servo_command(self, leg_index, motor_index, joint_angle):

        """
        Sends a servo command to the given channel on the pca by converting the
        joint angle to a duty cycle and sending it to the channel

        :param leg_index:
        :param motor_index:
        :param joint_angle:
        """
        ch = self.channels[motor_index]
        duty_cycle = angle_to_duty(joint_angle, self.pwm_params, self.servo_params, motor_index, leg_index)

        # Sets the duty cycle on the given channel
        if leg_index == 0:
            self.pca0.channels[ch].duty_cycle = duty_cycle
        elif leg_index == 1:
            self.pca1.channels[ch].duty_cycle = duty_cycle
        elif leg_index == 2:
            self.pca2.channels[ch].duty_cycle = duty_cycle
        elif leg_index == 3:
            self.pca3.channels[ch].duty_cycle = duty_cycle

    # Used in calibrate_servos.py to calibrate the ServoCalibration.py
    def set_actuator_position0(self, joint_angle, leg_index, motor_index):
        self.servo_command0(leg_index, motor_index, joint_angle)

    def servo_command0(self, leg_index, motor_index, joint_angle):
        """
        Sends a servo command to the given channel on the pca by converting the
        joint angle to a duty cycle and sending it to the channel

        :param leg_index:
        :param motor_index:
        :param joint_angle:
        """
        ch = self.channels[motor_index]
        duty_cycle = angle_to_duty0(joint_angle, self.pwm_params, self.servo_params, motor_index, leg_index)

        # Sets the duty cycle on the given channel
        if leg_index == 0:
            self.pca0.channels[ch].duty_cycle = duty_cycle
        elif leg_index == 1:
            self.pca1.channels[ch].duty_cycle = duty_cycle
        elif leg_index == 2:
            self.pca2.channels[ch].duty_cycle = duty_cycle
        elif leg_index == 3:
            self.pca3.channels[ch].duty_cycle = duty_cycle

    def get_imu_tilt(self):
        # Getting data from IMU
        accel = self.mpu.get_accel_data()
        gyro = self.mpu.get_gyro_data()

        gx = gyro['x']  # rotation about X-axis (°/s)
        gy = gyro['y']  # rotation about Y-axis (°/s)

        x = accel['x']
        y = accel['y']
        z = accel['z']

        # Roll in radians
        roll_rad = np.atan2(y, z)

        # Pitch in radians
        yz_dist = np.sqrt(y * y + z * z)
        pitch_rad = np.atan2(-x, yz_dist)

        # Convert to degrees
        roll_deg = np.degrees(roll_rad)
        pitch_deg = np.degrees(pitch_rad)

        return -roll_deg, -pitch_deg, -gx, -gy


def angle_to_duty(angle, pwm_params, servo_params, motor_index, leg_index):

    """
    Converts the given angle to a duty cycle for the given leg and motor

    :param angle: Desired angle from axis (in radians)
    :param pwm_params:
    :param servo_params:
    :param motor_index:
    :param leg_index:
    :return: duty cycle
    """

    # Finds the deviation from the neutral angle defined in ServoCalibration
    angle_deviation = (
        angle - servo_params.neutral_angles[motor_index, leg_index]
    ) * servo_params.servo_multipliers[motor_index, leg_index]
    
    if motor_index == 2:
        angle_deviation = 1.22*angle_deviation
    
    # Calculates the pulse width in µs
    pulse_width_micros = (
        servo_params.neutral_position_pwm[motor_index,leg_index]
        + servo_params.micros_per_rad[motor_index,leg_index] * angle_deviation
    )

    # Calculates duty cycle from pulse width, frequency and range
    return int(pulse_width_micros / 1e6 * pwm_params.freq * pwm_params.range)

def angle_to_duty0(angle, pwm_params, servo_params, motor_index, leg_index):

    """
    Converts the given angle to a duty cycle for the given leg and motor

    :param angle: Desired angle from axis (in radians)
    :param pwm_params:
    :param servo_params:
    :param motor_index:
    :param leg_index:
    :return: duty cycle
    """

    # Finds the deviation from the neutral angle defined in ServoCalibration
    angle_deviation = angle * servo_params.servo_multipliers[motor_index, leg_index]

    # Calculates the pulse width in µs
    pulse_width_micros = (
        servo_params.neutral_position_pwm[motor_index,leg_index]
        + servo_params.micros_per_rad[motor_index,leg_index] * angle_deviation
    )

    # Calculates duty cycle from pulse width, frequency and range
    return int(pulse_width_micros / 1e6 * pwm_params.freq * pwm_params.range)