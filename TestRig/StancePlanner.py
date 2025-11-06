import numpy as np

class StancePlanner:

    def __init__(self,config):
        self.config = config

    def touchdown_location(self):
        # TODO: Define values in Config
        return self.config.velocity*self.config.stancetime*0.5 # Raibert et al.

    def linear_discretizer(self):
        n = max(2,int(self.config.stancetime * self.config.frequency))
        x_uni=np.linspace(self.touchdown_location(),-self.touchdown_location(),n)
        z_uni=np.zeros(n) -self.config.Z_zero
        return x_uni, z_uni
