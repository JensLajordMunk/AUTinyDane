from HardwareInterface import HardwareInterface
import numpy as np

def get_motor_name(i, j):
    motor_type = {0: "abduction", 1: "hip", 2: "knee"}
    leg_pos = {0: "front-right", 1: "front-left", 2: "back-right", 3: "back-left"}
    final_name = motor_type[i] + " " + leg_pos[j]
    return final_name

def get_motor_setpoint(i, j):
    data = np.array([[0, 0, 0, 0], [45, 45, 45, 45], [45, 45, 45, 45]]) # TODO: Might have to change
    return data[i, j]

def degrees_to_radians(input_array):
    return input_array * np.pi / 180.0

def radians_to_degrees(input_array):
    return input_array * 180.0 / np.pi

def step_until(hardware_interface, leg, motor, set_point):

    found_position = False
    set_names = ["horizontal", "horizontal", "vertical"] ## TODO: Might have to change
    offset = 0

    while not found_position:
        move_input = str(
            input("Enter 'n' or 'm' to move the link until it is **" + set_names[motor] + "**. Enter 's' when done. Input: "
            )
        )
        if move_input == "n":
            offset += 1.0 # 1 degree added
            hardware_interface.set_actuator_position(
                degrees_to_radians(set_point + offset),
                leg,
                motor,
            )
        elif move_input == "m":
            offset -= 1.0
            hardware_interface.set_actuator_position(
                degrees_to_radians(set_point + offset),
                leg,
                motor,
            )
        elif move_input == "s":
            found_position = True
    return offset

def overwrite(servo_params):
    array_str = "np.array(\n ["
    for index,x in np.ndenumerate(servo_params.neutral_angle_degrees):
        row,col = index
        array_str += f"{x}"
        if col == 3:
            if row == 2:
                array_str += "])"
            else:
                array_str += "],\n ["
        else:
            array_str+=", "

    file_contents = f"""# This file was automatically written using calibrate_servos.py, do not change manually
import numpy as np
class ServoCalibration:
    def __init__(self):
        self.MICROS_PER_RAD = {servo_params.micros_per_rad}
        self.NEUTRAL_ANGLE_DEGREES = {array_str}"""
    with open("ServoCalibration.py", "w") as f:
        f.write(file_contents)

def calibrate_angle_offset(hardware_interface):

    hardware_interface.servo_params.neutral_angle_degrees = np.zeros((3, 4))
    for leg in range(4):
        for motor in range(3):
            motor_name = get_motor_name(motor, leg)
            print("\nCalibrating the **" + motor_name + " motor **")
            set_point = get_motor_setpoint(motor, leg)

            # Zero out the neutral angle
            hardware_interface.servo_params.neutral_angle_degrees[motor, leg] = 0

            # Move servo to set_point angle
            hardware_interface.set_actuator_position(
                degrees_to_radians(set_point),
                leg,
                motor
            )

            offset = step_until(
                hardware_interface, leg, motor, set_point
            )

            # TODO: Figure this out: This depends on our robot and is to be determined when seeing the direction of angles
            if motor == 1:
                hardware_interface.servo_params.neutral_angle_degrees[motor, leg] = set_point - offset
            else:
                hardware_interface.servo_params.neutral_angle_degrees[motor, leg] = -(set_point + offset)

            # The servo should now have the correct micros_per_rad and should move to the correct positions
            hardware_interface.set_actuator_position(
                degrees_to_radians([0, 45, -45][motor]), # TODO: Might have to change
                leg,
                motor,
            )

def main():
    hardware_interface = HardwareInterface()
    calibrate_angle_offset(hardware_interface)
    overwrite(hardware_interface.servo_params)

main()