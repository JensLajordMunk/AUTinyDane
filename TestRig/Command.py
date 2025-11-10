import numpy as np

class Command:
    def __init__(self,config):
        self.config = config

        #-------------- Controller parameters ----------------
        self.L3 = [0.5, 0]
        self.R3 = [0, 0]

        self.velocity = self.config.max_velocity * self.L3[0]
        self.yaw_rate = self.config.max_yaw_rate * self.L3[1]
        self.pitch = self.config.max_pitch * self.R3[0]
        self.roll = self.config.max_roll* self.R3[1]