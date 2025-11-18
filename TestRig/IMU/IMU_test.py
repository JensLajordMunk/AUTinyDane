import time
import board
import adafruit_mpu6050

i2c = board.I2C()  # This automatically uses I2C bus 1 (SDA=GPIO2, SCL=GPIO3)

mpu = adafruit_mpu6050.MPU6050(i2c)

while True:
    print(f"Acceleration: {mpu.acceleration}")
    print(f"Gyro: {mpu.gyro}")
    print(f"Temperature: {mpu.temperature} °C")
    print()
    time.sleep(1)
