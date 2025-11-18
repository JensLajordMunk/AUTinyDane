import time
import board
import adafruit_mpu6050
import types
import busio


brd = types.SimpleNamespace(SCL=11, SDA=10)
i2c = busio.I2C(brd.SCL, brd.SDA)

mpu = adafruit_mpu6050.MPU6050(i2c)

while True:
    print(f"Acceleration: {mpu.acceleration}")
    print(f"Gyro: {mpu.gyro}")
    print(f"Temperature: {mpu.temperature} °C")
    print()
    time.sleep(1)
