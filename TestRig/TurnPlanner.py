import numpy as np
from SwingPlanner import SwingPlanner
from StancePlanner import StancePlanner
class TurnPlanner:

    def __init__(self,config):
        self.config = config
        self.swing_planner = SwingPlanner(config)
        self.stance_planner = StancePlanner(config)

    def Radiuses(self):
        inner_radius = self.config.min_turn_radius/ abs(self.config.control_y)
        outer_radius = inner_radius + self.config.body_width + 2*self.config.abduction_offset
        return inner_radius, outer_radius

    def Y_pos(self):
        innerR, outerR = self.Radiuses()
        # Right turn:
        if self.config.control_y > 0:
            td = self.swing_planner.touchdown_location()

            theta_right = np.arcsin(td,innerR)
            y_right = innerR - np.cos(theta_right)*innerR

            theta_left = np.arcsin(td,outerR)
            y_left = outerR - np.cos(theta_left)*outerR

        # Left turn
        if self.config.control_y < 0:
            td = self.swing_planner.touchdown_location()

            theta_left = np.arcsin(td, innerR)
            y_left = innerR - np.cos(theta_left) * innerR

            theta_right = np.arcsin(td, outerR)
            y_right = outerR - np.cos(theta_right) * outerR

        return y_right, y_left