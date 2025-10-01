import pigpio
from Configuration import PWMParams, ServoParams

class HardwareInterface:
    def __init__(self):
        self.pi = pigpio.pi()

        if not self.pi.connected:
            raise RuntimeError("pigpio could not connect to RPi pins")

        self.pwm_params = PWMParams()
        self.servo_params = ServoParams()

        # Define pin parameters for each pin
        define_pin_parameters(self.pi, self.pwm_params)


    def set_actuator_positions(self, joint_angles):
        for leg_index in range(4):
            for motor_index in range(3):
                servo_command(self.pi, self.pwm_params, self.servo_params, leg_index, motor_index, joint_angles[motor_index,leg_index])


    def set_actuator_position(self, joint_angles, leg_index):
        for m_idx in range(3):
            servo_command(self.pi, self.pwm_params, self.servo_params, leg_index, m_idx, joint_angles[m_idx])


def define_pin_parameters(pi, pwm_params):

    """
    Takes in a pigpio.pi object and a PWMParams object and defines the PWM frequency and range for each pin.

    :param pi: pigpio object which talks to the pins on the RPi
    :param pwm_params: PWM parameters which is send to the pins given and the corresponding frequency and range
    """

    for leg_index in range(4):
        for motor_index in range(3):
            # Sets the PWM frequency for each pin corresponding to the leg and motor index
            # and the pwm frequency in Configuration
            pi.set_PWM_frequency(int(pwm_params.pins[motor_index, leg_index]), pwm_params.freq)
            # Sets PWM range from configuration for the motor and pin
            pi.set_PWM_range(int(pwm_params.pins[motor_index, leg_index]), pwm_params.range)


def servo_command(pi, pwm_params, servo_params, leg_index, motor_index, joint_angle):

    """
    Sends a servo command to the given pin on the RPi by converting the
    joint angle to a duty cycle and sending it to the pin

    :param pi: pigpio object
    :param pwm_params:
    :param servo_params:
    :param leg_index:
    :param motor_index:
    :param joint_angle:
    """

    duty_cycle = angle_to_duty(joint_angle, pwm_params, servo_params, motor_index, leg_index)

    # Sets the duty cycle on the given pin
    pi.set_PWM_dutycycle(int(pwm_params.pins[motor_index, leg_index]), duty_cycle)


def angle_to_duty(angle, pwm_params, servo_params, motor_index, leg_index):

    """
    Converts the given angle to a duty cycle for the given pin

    :param angle:
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
        servo_params.neutral_position_pwm
        + servo_params.micros_per_rad * angle_deviation
    )

    # Calculates duty cycle from pulse width, frequency and range
    return int(pulse_width_micros / 1e6 * pwm_params.freq * pwm_params.range)
