from enum import Enum

class Modes(Enum):
    NEUTRAL = 0
    TROT = 1
    ROTATE = 2
    TRANSLATE = 3

class Command:
    def __init__(self, config):
        self.config = config

        # -------------- Controller input ----------------
        self.L3 = [0, 0]
        self.R3 = [0, 0]
        self.controller_connected = False

        # -------------- Controller parameters ----------------

        self.yaw_array = []
        self.pitch_array = []
        self.roll_array = []
        self.mode = Modes.NEUTRAL

    @property
    def stand_yaw(self):
        return self.config.max_yaw_stand * self.L3[0]

    @property
    def trot_yaw(self):
        return self.config.max_yaw_rate * self.R3[0]

    @property
    def stand_pitch(self):
        return self.config.max_pitch * self.R3[1]

    @property
    def stand_roll(self):
        return self.config.max_roll* self.R3[0]

    @property
    def stand_z(self):
        return self.config.max_z * self.L3[1]

    @property
    def stand_x(self):
        return self.config.max_x * self.R3[1]

    @property
    def stand_y(self):
        return self.config.max_y* self.R3[0]

    @property
    def velocityX(self):
        return 0.75 * self.config.max_velocityX * self.L3[1]

    @property
    def velocityY(self):
        return 0.75 * self.config.max_velocityY * self.L3[0]

