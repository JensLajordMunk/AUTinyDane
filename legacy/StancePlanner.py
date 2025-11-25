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

    def linear_discretizer(self):
        #################### READ ME ######################
        """ Discretizes stance as a linear line between front and back position,
            but if the velocity is zero changes to constant"""

        n = max(2,int(self.config.stancetime * self.config.frequency))
        if round(self.state.velocityX + self.state.velocityY,10) == 0.0:
            x_uni = np.zeros(n)
            y_uni = np.zeros(n)
            z_uni = np.zeros(n) - self.config.body_height
        else:
            TDX, TDY = self.touchdown_location()
            x_uni=np.linspace(TDX,-TDX,n)
            y_uni = np.linspace(TDY, -TDY, n)
            z_uni=np.zeros(n) -self.config.body_height
        return x_uni, y_uni, z_uni

    def linear_discretizer_manual(self, Xbegin, Ybegin,n):
        #################### READ ME ######################
        """ Discretizes for changing conditions, by manually setting starting position"""

        if round(self.state.velocityX + self.state.velocityY,10) == 0:
            x_uni = np.zeros(n)
            y_uni = np.zeros(n)
            z_uni = np.zeros(n) - self.config.body_height
        else:
            TDX, TDY = self.touchdown_location()
            x_uni = np.linspace(Xbegin,-TDX,n)
            y_uni = np.linspace(Ybegin, -TDY, n)
            z_uni=np.zeros(n) -self.config.body_height
        return x_uni, y_uni, z_uni
