from mpu6050 import mpu6050

""" MAKE SURE IT IS ON A FLAT SURFACE!!!"""

sensor = mpu6050(0x68)
N = 4000 # Change depending preference
ax, ay, az, gx, gy, gz = 0, 0, 0, 0, 0, 0
input("Make sure the robot is on a completely flat surface. Proceed by button press")

for _ in range(N):
    a = sensor.get_accel_data()
    g = sensor.get_gyro_data()

    ax += a['x']
    ay += a['y']
    az += a['z']
    gx += g['x']
    gy += g['y']
    gz += g['z']

print(f"self.accel_offset = {{'x': {ax / N:.3f}, 'y': {ay / N:.3f}, 'z': {az / N - 9.82:.3f}}}")
print(f"self.gyro_offset  = {{'x': {gx / N:.3f}, 'y': {gy / N:.3f}, 'z': {gz / N:.3f}}}")
print("Change this in the constructor of HardwareInterface.py")