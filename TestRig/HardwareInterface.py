import numpy as np
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from Configuration import PWMParams, ServoParams
 
class HardwareInterface:
    def __init__(self):
        self.pwm_params = PWMParams()
        self.servo_params = ServoParams()

        self.i2c = busio.I2C(SCL, SDA)
        self.pca0 = PCA9685(self.i2c, address=0x40)
        self.pca0.frequency = self.pwm_params.freq
        #self.pca1 = PCA9685(self.i2c, address=0x41)
        #self.pca1.frequency = self.pwm_params.freq
        self.pca2 = PCA9685(self.i2c, address=0x42)
        self.pca2.frequency = self.pwm_params.freq

        self.channels = [0, 1, 2]

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
        #elif leg_index == 1:
        #    self.pca1.channels[ch].duty_cycle = duty_cycle
        elif leg_index == 1:
            self.pca2.channels[ch].duty_cycle = duty_cycle
        #elif leg_index == 3:
        #    self.pca3.channels[ch].duty_cycle = duty_cycle

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
        #elif leg_index == 1:
        #    self.pca1.channels[ch].duty_cycle = duty_cycle
        elif leg_index == 1:
            self.pca2.channels[ch].duty_cycle = duty_cycle
        #elif leg_index == 3:
        #    self.pca3.channels[ch].duty_cycle = duty_cycle

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