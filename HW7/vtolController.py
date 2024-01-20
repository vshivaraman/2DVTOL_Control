import numpy as np
from vtolDynamics import vtolDynamics

vtol = vtolDynamics()

class vtolController:
    
    def __init__(self):
        self.Kp = 0.154275
#        self.Kd = 0.79325
        self.rotForce_limit = 15

    def update(self,height_ref,state_height):
        h = state_height[0]
        h_ = state_height[1]
        force_min = (vtol.mass_centerpod+2*vtol.mass_rotor)*vtol.g
        force = force_min + self.Kp*(height_ref - h)#-self.Kd*h_)

        rotForce = [force/2,force/2]

        rotForce = self.saturate(rotForce)

        return rotForce

    def saturate(self,rotForce):

        if abs(rotForce[0]) > self.rotForce_limit:
            rotForce[0] = self.rotForce_limit
        if abs(rotForce[1]) > self.rotForce_limit:
            rotForce[1] = self.rotForce_limit

        return rotForce
