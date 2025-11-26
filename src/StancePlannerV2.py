import numpy as np

class StancePlanner:

    def __init__(self,state,config):
        self.state = state
        self.config = config

    def touchdown_location(self):
        #################### READ ME ######################
        """ This i based on Raiberts touchdown location, and calculates how far the
            foot has drifted backwards during the stance and is placed the same distance
            forward in the swing phase"""

        TDX = self.state.velocityX * self.config.stancetime * 0.5
        TDY = self.state.velocityY * self.config.stancetime * 0.5
        return TDX, TDY # Raibert et al.

    def linear_discretizer(self, time):
        #################### READ ME ######################
        """ Discretizes stance as a linear line between front and back position,
            but if the velocity is zero changes to constant"""

        if round(abs(self.state.velocityX) + abs(self.state.velocityY),10) == 0.0:
            x = 0
            y = 0
            z = - self.config.body_height
        else:
            TDX, TDY = self.touchdown_location()
            start = np.array([TDX,TDY,-self.config.body_height])
            end = np.array([-TDX,-TDY,-self.config.body_height])
            ratio = time/self.config.stancetime
            p = start + (end - start) * ratio
            x, y, z = p[0], p[1], p[2]
        return x, y, z

    def stance_bezier_discretizer(self, time):

        ratio = time/self.config.stancetime
        t = np.clip(ratio, 0.0, 1.0)

        TDX, TDY = self.touchdown_location()
        x = TDX + (-TDX - TDX)*t
        y = TDY + (-TDY - TDY) * t

        P0 = 0
        P1 = 0
        P2 = 0
        P3 = 0.004
        P4 = -0.02
        P5 = 0.001
        P6 = 0

        u = 1 - t
        z = (
            (1  * u**6 * t**0 * P0) +
            (6  * u**5 * t**1 * P1) +
            (15 * u**4 * t**2 * P2) +
            (20 * u**3 * t**3 * P3) +
            (15 * u**2 * t**4 * P4) +
            (6  * u**1 * t**5 * P5) +
            (1  * u**0 * t**6 * P6)
        )

        return x, y, z-self.config.body_height
