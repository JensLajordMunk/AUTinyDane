# Install on Raspberry Pi:
# sudo apt-get update
#sudo apt-get install -y python3-pip
# Low-level: pip3 install --upgrade adafruit-circuitpython-pca9685 adafruit-circuitpython-motor adafruit-blinka
# High-level: pip3 install adafruit-circuitpython-servokit

# High-level:
from adafruit_servokit import ServoKit
import time

# TODO: If possible inherit functions, methods and classes from existing files to lower complexity

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

#if __name__ == "__main__":
    # Run only ONE test by calling the function
    #test_servo_basic()


# Low-level:
# Purpose: Drive servos via PCA9685 using your own angle→pulse mapping.
# Library: adafruit-circuitpython-pca9685 (uses 16-bit duty_cycle on channels)
import time
import numpy as np
import board
import busio
from adafruit_pca9685 import PCA9685
from HardwareInterface import pwm_params

# ---------------------------
# Global PCA9685 settings
# ---------------------------
I2C_ADDR = 0x40
PWM_FREQ_HZ = 330      #Servo frequency

# ---------------------------
# Per-channel servo calibration
# Each entry is a dict for a PCA9685 CHANNEL index (0..15):
#   - neutral_angle_rad: mechanical "zero" for this joint (radians)
#   - neutral_pulse_us:  pulse width (µs) that holds that neutral angle
#   - micros_per_rad:    slope: µs per radian around neutral
#   - min_us/max_us:     hard limits to protect the servo/mechanism
# ---------------------------
DEFAULT_CAL = {
    "neutral_angle_rad": 0.0,
    "neutral_pulse_us": 1520.0,
    # If servo is ~500..2500 µs for ~±90° (i.e., 180° total), Test this
    # slope ≈ (2500-500) / π  ≈ 636.62 µs/rad
    "micros_per_rad": (2020.0 - 1000.0) / np.pi,
    "min_us": 1000.0,
    "max_us": 2020.0,
}

CHANNEL_CALS = {
    ch: DEFAULT_CAL.copy() for ch in range(16)
}
# Example of per-joint tweaks (uncomment and tune as you calibrate):
# CHANNEL_CALS[0]["neutral_angle_rad"] = math.radians(+2.0)
# CHANNEL_CALS[0]["neutral_pulse_us"]  = 1490.0
# CHANNEL_CALS[0]["micros_per_rad"]    = 640.0
# CHANNEL_CALS[0]["min_us"]            = 520.0
# CHANNEL_CALS[0]["max_us"]            = 2400.0


# ---------------------------
# Mapping functions
# ---------------------------
def angle_rad_to_pulse_us(angle_rad: float, cal: dict) -> float:
    """
    Convert a joint angle (radians) to a pulse width in microseconds
    using per-channel calibration parameters.
    """
    angle_dev = angle_rad - cal["neutral_angle_rad"]
    pulse_us = cal["neutral_pulse_us"] + cal["micros_per_rad"] * angle_dev
    # Clamp to safe limits
    pulse_us = np.clip(pulse_us, cal["min_us"], cal["max_us"])
    return pulse_us


def pulse_us_to_duty16(pulse_us: float, pwm_params: float) -> int:
    """
    Convert a pulse width (µs) to a 16-bit duty cycle (0..65535)
    for the CircuitPython PCA9685 'duty_cycle' API.
    """
    duty = int(pulse_us / 1e6 * pwm_params.freq * pwm_params.range)
    # Bound to 16-bit range
    duty = np.clip(pulse_us, 0, pwm_params.range)
    return duty


def set_servo_angle_deg(pca: PCA9685, channel: int, angle: float):
    """
    Convenience: take angle in degrees, map to µs, then to duty16, and write.
    """
    cal = CHANNEL_CALS[channel]
    pulse_us = angle_rad_to_pulse_us(angle, cal)
    duty = pulse_us_to_duty16(pulse_us, pca.frequency)
    pca.channels[channel].duty_cycle = duty


# ---------------------------
# Simple motion helpers
# ---------------------------
def move_to(pca: PCA9685, channel: int, angle_deg: float, hold_s: float = 0.0):
    """Move a servo to an absolute angle (deg) and optionally hold for some seconds."""
    set_servo_angle_deg(pca, channel, angle_deg)
    if hold_s > 0:
        time.sleep(hold_s)
