from HardwareInterface import HardwareInterface
import time

def main():
    hardwareinterface = HardwareInterface()

    print("IMU test running... Move the robot to see pitch/roll changes.\n")

    while True:
        try:
            pitch, roll = hardwareinterface.get_imu_tilt()
            print(f"Pitch: {pitch:.2f}°   Roll: {roll:.2f}°")
            time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nIMU test stopped.")
            break

def main1():
    hardwareinterface = HardwareInterface()

    accel = hardwareinterface.sensor.get_accel_data()
    gyro = hardwareinterface.sensor.get_gyro_data()
    temp = hardwareinterface.sensor.get_temp()

    print(accel)
    print(gyro)
    print(temp)


main1()
