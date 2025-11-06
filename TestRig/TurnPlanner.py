import numpy as np
from SwingPlanner import Swing
class TurnPlanner:

    def __init__(self,config):
        self.config = config

    def Radiuses(self):
        inner_radius = self.config.min_turn_radius/ abs(self.config.control_y)
        outer_radius = inner_radius + self.config.body_width + 2*self.config.abduction_offset
        return inner_radius, outer_radius

    def SwingY(self):
        # Right turn:
        if self.config.control_y > 0:
            td =
            theta_left =


    def StanceY(self):

