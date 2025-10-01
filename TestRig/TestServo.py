# Install on Raspberry Pi:
# sudo apt-get update
#sudo apt-get install -y python3-pip
# Low-level: pip3 install --upgrade adafruit-circuitpython-pca9685 adafruit-circuitpython-motor adafruit-blinka
# High-level: pip3 install adafruit-circuitpython-servokit

# High-level:
from adafruit_servokit import ServoKit
import time

kit = ServoKit(channels=16)

def test_servo_basic():
    """Move servo on channel 0 through a few positions"""
    kit.servo[0].angle = 90
    time.sleep(1)
    kit.servo[0].angle = 0
    time.sleep(1)
    kit.servo[0].angle = 180
    time.sleep(1)

def test_servo_sweep():
    """Sweep servo from 0 to 180 and back"""
    for angle in range(0, 181, 10):
        kit.servo[0].angle = angle
        time.sleep(0.05)
    for angle in range(180, -1, -10):
        kit.servo[0].angle = angle
        time.sleep(0.05)

if __name__ == "__main__":
    # Run only ONE test by calling the function
    #test_servo_basic()


from HardwareInterface import angle_to_duty


angle_to_duty(angle, pwm_params, servo_params, motor_index, leg_index)