import numpy as np
import control as ctl
from math import pi
from vtolDynamics import vtolDynamics
import vtolParam as P

vtol = vtolDynamics()

class vtolController:
    
    def __init__(self):
        self.rotForce_limit = 15
        self.integrator_h = 0.0
        self.integrator_z = 0.0
        self.error_dl_h = 0.0
        self.error_dl_z = 0.0
        self.K_h = P.K_h
        self.ki_h = P.ki_h
        self.K_z = P.K_z
        self.ki_z = P.ki_z

    def update(self,z_ref,height_ref,state):
        z = state.item(0)
        h = state.item(1)
        theta = state.item(2)
        z_ = state.item(3)
        h_ = state.item(4)
        theta_ = state.item(5)

        force_min = (vtol.mass_centerpod + 2*vtol.mass_rotor)*vtol.g
        error_h = height_ref-h
        self.integrateError_h(error_h)
        force = force_min - self.K_h@np.array([[h],[h_]]) + self.ki_h*self.integrator_h
        error = z_ref-z
        self.integrateError_z(error)
        tau = -self.K_z@np.array([[z],[theta],[z_],[theta_]]) + self.ki_z*self.integrator_z
        force = float(force)
        tau = float(tau)


        rotForceLeft = force/2 - tau/(2*vtol.rod_length)
        rotForceRight = force/2 + tau/(2*vtol.rod_length)

        rotForce = [rotForceLeft,rotForceRight]
        rotForce = self.saturate(rotForce)
        
        return rotForce

    def integrateError_h(self,error):
        self.integrator_h = self.integrator_h + (0.01/2.0)*(error+self.error_dl_h)
        self.error_dl_h = error

    def integrateError_z(self,error):
        self.integrator_z = self.integrator_z + (0.01/2.0)*(error+self.error_dl_z)
        self.error_dl_z = error

    def saturate(self,rotForce):

        if abs(rotForce[0]) > self.rotForce_limit:
            rotForce[0] = self.rotForce_limit
        if abs(rotForce[1]) > self.rotForce_limit:
            rotForce[1] = self.rotForce_limit

        return rotForce



