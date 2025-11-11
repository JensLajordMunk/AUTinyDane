from PS4Controller.controllerInput import controller_listen, controller_stop
import serial
import time
import numpy as np
from Configuration import RobotConfig
from Kinematics import inverse_kinematics
from HardwareInterface import HardwareInterface
import Command
import State
# ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null

def test_leg_movement():
    # --- setup ---
    hardware_interface = HardwareInterface()
    configuration = RobotConfig()
    leg_index = int(input("Enter leg index (0-3): "))

    # --- serial setup to Arduino ---
    # Adjust port if necessary (check using: ls /dev/ttyACM*)
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

    # --- constants ---
    y_fixed = 0.04              # [m]
    x_range = (-0.03, 0.05)     # backward → forward
    z_range = (-0.12, -0.045)    # down → up
    update_hz = 50
    dt = 1.0 / update_hz

    print("Controlling leg via joystick (Ctrl+C to stop)...")

    def unit_to_range(u, lo, hi):
        """Map from [-1,1] to [lo,hi]."""
        return lo + (u + 1.0) * 0.5 * (hi - lo)

    while True:
        try:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue

            # Arduino sends e.g. "123,-420"
            ux_promille, uz_promille = map(int, line.split(","))

            # normalize to [-1,1]
            ux = np.clip(ux_promille / 1000.0, -1.0, 1.0)
            uz = np.clip(uz_promille / 1000.0, -1.0, 1.0)

            # optionally invert directions if joystick feels reversed
            # ux = -ux
            # uz = -uz

            # scale to physical coordinates
            x = unit_to_range(ux, *x_range)
            y = y_fixed
            z = unit_to_range(uz, *z_range)

            # compute inverse kinematics
            try:
                target_angles = inverse_kinematics(np.array([x, y, z]), leg_index, configuration)
            except ValueError as e:
                print(f"Inverse kinematics error: {e}")
                continue

            # send angles to the actuators (1 leg = 3 motors)
            for motor_index in range(3):
                hardware_interface.set_actuator_position(
                    target_angles[motor_index], leg_index, motor_index
                )

            # optional: print status
            print(f"x={x:+.3f} z={z:+.3f} -> [abd, hip, knee] = {np.degrees(target_angles)} deg")

            time.sleep(dt)

        except KeyboardInterrupt:
            print("\nStopping leg control.")
            break
        except Exception as e:
            print("Serial/parse error:", e)

def controller_movement(config, command, state):
    leg_index = int(input("Enter leg index (0-3): "))
    # --- constants ---
    y_fixed = 0.04              # [m]
    x_range = (-0.03, 0.05)     # backward → forward
    z_range = (-0.045, -0.12)    # down → up
    update_hz = 50
    dt = 1.0 / update_hz

    hardware_interface = HardwareInterface()
    print("Thread started.")
    control_panel = controller_listen(commandConfig=testCommand)
    y = 0.04
    x = 0


    while True:
        try:
            z = np.interp(command.L3[1], (-1, 1), z_range)

            # compute inverse kinematics
            try:
                target_angles = inverse_kinematics(np.array([x, y, z]), leg_index, config)
            except ValueError as e:
                print(f"Inverse kinematics error: {e}")
                pass

            # send angles to the actuators (1 leg = 3 motors)
            for motor_index in range(3):
                hardware_interface.set_actuator_position(
                    target_angles[motor_index], leg_index, motor_index
                )

            # optional: print status
            print(f"x={x:+.3f} z={z:+.3f} -> [abd, hip, knee] = {np.degrees(target_angles)} deg")

            time.sleep(dt)
        except Exception as e:
            print(Exception)
            break


    

if __name__ == "__main__":
    testConfig = RobotConfig()
    testCommand = Command.Command(testConfig)
    testState = State.State(testConfig)

    controller_movement(testConfig, testCommand, testState)


