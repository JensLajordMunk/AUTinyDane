import numpy as np

class InertiaBalancer:

    def __init__(self,config):

        self.config = config

        self.kp_pitch = 0.
        self.kp_roll = 0.

        self.kd_pitch = 0.
        self.kd_roll = 0.

    def velocity_offset(self, dt, pitch, roll, gx, gy):
        pitch, roll, gx, gy = np.radians(pitch), np.radians(roll), np.radians(gx), np.radians(gy)

        accel_x = (self.kp_pitch * pitch) + (self.kd_pitch * gy)
        accel_y = -(self.kp_roll * roll) - (self.kd_roll * gx)

        accel_x = np.clip(accel_x, -self.config.max_acceleration,self.config.max_acceleration)
        accel_y = np.clip(accel_y, -self.config.max_acceleration,self.config.max_acceleration)

        vx_offset = dt * accel_x
        vy_offset = dt * accel_y

        return vx_offset, vy_offset
