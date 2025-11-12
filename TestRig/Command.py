import numpy as np

class Command:
    def __init__(self, config):
        self.config = config

        # -------------- Controller input ----------------
        self.L3 = [1, 1]
        self.R3 = [0, 0]
        self.controller_connected = False

        # -------------- Controller parameters ----------------


        #self.yaw_rate = self.config.max_yaw_rate * self.L3[1]
        #self.pitch = self.config.max_pitch * self.R3[0]
        #self.roll = self.config.max_roll* self.R3[1]

    @property
    def velocityX(self):
        return self.config.max_velocityX * self.L3[0]

    @property
    def velocityY(self):
        return self.config.max_velocityY * self.L3[1]

