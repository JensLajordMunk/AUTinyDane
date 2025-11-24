import numpy as np


class InertiaBalancer:

    def __init__(self,config):

        self.config = config

        self.kp_pitch = 10
        self.kp_roll = 0.05

        self.kd_pitch = 10
        self.kd_roll = 0.002

    def velocity_offset(self, dt, pitch, roll, gx, gy):
        pitch, roll, gx, gy = np.radians(pitch), np.radians(roll), np.radians(gx), np.radians(gy)

        accel_x = -(self.kp_pitch * pitch) - (self.kd_pitch * gy)
        accel_y = -(self.kp_roll * roll) - (self.kd_roll * gx)
        vx_offset = dt * accel_x
        vy_offset = dt * accel_y

        return vx_offset, vy_offset