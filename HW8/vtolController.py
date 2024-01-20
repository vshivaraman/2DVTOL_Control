import numpy as np
from math import pi
from vtolDynamics import vtolDynamics

vtol = vtolDynamics()

class vtolController:
    
    def __init__(self): # Constants
        self.Kp_h = 0.154275
        self.Kd_h = 0.793254
        self.Kp_theta = 0.365541
        self.Kd_theta = 0.187785
        self.Kp_z = 0.0077
        self.Kd_z = 0.034141
        self.rotForce_limit = 15

    def update(self,z_ref,height_ref,state):
        z = state.item(0)
        h = state.item(1)
        theta = state.item(2)
        z_ = state.item(3)
        h_ = state.item(4)
        theta_ = state.item(5)

        force_min = (vtol.mass_centerpod + 2*vtol.mass_rotor)*vtol.g
        force = force_min + (self.Kp_h*(height_ref-h) - self.Kd_h*h_)
        theta_ref = (self.Kp_z*(z_ref-z)-self.Kd_z*z_)
        torque = -self.Kp_theta*(theta_ref+theta)-self.Kd_theta*theta_


        rotForceLeft = force/2 - torque/(2.*vtol.rod_length)
        rotForceRight = force/2 + torque/(2.*vtol.rod_length)

        rotForce = [rotForceLeft,rotForceRight]
#        rotForce = self.saturate(rotForce)
        
        return rotForce

    def saturate(self,rotForce):

        if abs(rotForce[0]) > self.rotForce_limit:
            rotForce[0] = self.rotForce_limit
        if abs(rotForce[1]) > self.rotForce_limit:
            rotForce[1] = self.rotForce_limit

        return rotForce



