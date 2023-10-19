import numpy as np
from math import pi
import sys
from vtolDynamics import vtolDynamics
from PIDControl import PIDControl

vtol = vtolDynamics()

class vtolController:
    
    def __init__(self):
        self.Kp_h = 0.1542
        self.Kd_h = 0.79152
        self.Kp_theta = 0.365541
        self.Kd_theta = 0.187954
        self.Kp_z = 0.044141#0.0077
        self.Kd_z = 0.095#0.034141
        self.Ki_z = 0.0045
        self.rotForce_limit = 12.5
        self.zCtrl = PIDControl(self.Kp_z,self.Ki_z,self.Kd_z,0.05,0.05,0.01)
        self.thetaCtrl = PIDControl(self.Kp_theta,0.00,self.Kd_theta,self.rotForce_limit,0.05,0.01)

    def update(self,z_ref,height_ref,state):
        z = state.item(0)
        h = state.item(1)
        theta = state.item(2)
        z_ = state.item(3)
        h_ = state.item(4)
        theta_ = state.item(5)
        force_min = (vtol.mass_centerpod+2*vtol.mass_rotor)*vtol.g
        force = force_min + (self.Kp_h*(height_ref - h)-self.Kd_h*h_)
        theta_ref = self.zCtrl.PID(z_ref,z,False)
        tau = self.thetaCtrl.PD(theta_ref,theta,False)# Since the steady state error is 0 using only PD controller
        rotForceLeft = (1/2)*(force-tau/vtol.rod_length)
        rotForceRight = (1/2)*(force+tau/vtol.rod_length)
        
        rotForce = [rotForceLeft,rotForceRight]
        rotForce = self.saturate(rotForce)

        return rotForce

    def saturate(self,rotForce):

        if abs(rotForce[0]) > self.rotForce_limit:
            rotForce[0] = self.rotForce_limit
        if abs(rotForce[1]) > self.rotForce_limit:
            rotForce[1] = self.rotForce_limit

        return rotForce
