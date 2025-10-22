import numpy as np
import sympy as sp
import time
from HardwareInterface import HardwareInterface
from Configuration import RobotConfig
from Kinematics import inverse_kinematics
from State import State

class SwingPlanner:

    def __init__(self):
        self.config = config

    def touchdown_location(self):
        # TODO: Define values in Config
        return self.config.velocity*self.config.stancetime*0.5 # Raibert et al.

    def theta(self):
        return np.arctan2(2*(self.config.step_height + 2*self.config.arcR), self.config.velocity*self.config.stancetime)

    def key_points(self):
        x1 = -self.config.arcR*np.cos(self.theta()) - self.touchdown_location
        x2 = -self.config.arcR*np.cos(self.theta())
        y1 = self.config.arcR + np.sqrt(self.config.arcR**2 - (x1 + self.touchdown_location)**2)
        y2 = self.config.step_height - self.config.arcR + self.config.arcR*np.sin(self.theta())
        return x1, x2, y1, y2

    def bottom_arc_length(self): #Symmetry gives that the two bottom arcs are of equal length
        return 2*0.25*self.config.arcR*np.pi

    def transition_arc_length(self):
        return self.config.arcR*self.theta()

    def linear_section_length(self):
        x1, x2, y1, y2 = self.key_points()
        return np.sqrt( (x2-x1)**2 + (y2-y1)**2 )

    def half_top_arc_length(self):
        return (np.pi*0.5 + self.theta())*self.config.arcR

    def total_swing_length(self):
        return 2 * (self.bottom_arc_length() + self.transition_arc_length() + self.linear_section_length() + self.half_top_arc_length())

    def section_phase_times(self):
        bottom_arc_time = self.config.swingtime*self.bottom_arc_length()/self.total_swing_length()
        transitional_arc_time = self.config.swingtime*self.transition_arc_length()/self.total_swing_length()
        linear_length_time = self.config.swingtime*self.linear_section_length()/self.total_swing_length()
        top_arc_time = self.config.swingtime*2*self.half_top_arc_length/self.total_swing_length()
        return bottom_arc_time, transitional_arc_time, linear_length_time, top_arc_time

    def function_discretizer(self, sympy_func, duration):
        t = sp.Symbol('x')
        numpy_func = sp.lambdify(t, sympy_func, 'numpy')

        n = int(duration * self.config.frequency)
        x_vals = np.linspace(0, duration, n, endpoint=False)
        y_vals = numpy_func(x_vals)
        return x_vals, y_vals

    def m(self):
        x1, x2, y1, y2 = self.key_points()
        return (y2 - y1)/(x2 - x1)

    def functions(self):
        x = sp.Symbol('x')
        f1 = self.config.arcR - np.sqrt(self.config.arcR**2 - (x + self.touchdown_location())**2)
        f2 = self.config.arcR + sqrt(self.config.arcR**2 - (x + self.touchdown_location())**2)
        x1, x2, y1, y2 = self.key_points()
        f3 = self.m()*(x - x1) + y1
        f4 = self.config.step_height - self.config.arcR + np.sqrt(self.config.arcR**2 - x**2)
        f5 = -self.m()*(x + x1) + y1
        # TODO: There is an underlying problem in not storing the limits of the functions. Figure this out first
