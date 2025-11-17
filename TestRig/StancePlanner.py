import numpy as np

class StancePlanner:

    def __init__(self,state,config):
        self.state = state
        self.config = config

    def touchdown_location(self):
        # TODO: Define values in Config
        TDX = self.state.velocityX * self.config.stancetime * 0.5
        TDY = self.state.velocityY * self.config.stancetime * 0.5
        return TDX, TDY # Raibert et al.

    def linear_discretizer(self):
        n = max(2,int(self.config.stancetime * self.config.frequency))
        if self.state.velocityX + self.state.velocityY == 0:
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
        if self.state.velocityX + self.state.velocityY == 0:
            x_uni = np.zeros(n)
            y_uni = np.zeros(n)
            z_uni = np.zeros(n) - self.config.body_height
        else:
            TDX, TDY = self.touchdown_location()
            x_uni = np.linspace(Xbegin,-TDX,n)
            y_uni = np.linspace(Ybegin, -TDY, n)
            z_uni=np.zeros(n) -self.config.body_height
        return x_uni, y_uni, z_uni
