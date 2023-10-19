import numpy as np
import control as ctl
from math import pi
from vtolDynamics import vtolDynamics

vtol = vtolDynamics()

class vtolController:
    
    def __init__(self): 
        self.rotForce_limit = 13
        # Longitudinal Constants
        self.A_h = np.array([[0.0,1.0],[0.0,0.0]])
        self.B_h = np.array([[0.0],[vtol.mass_centerpod+2*vtol.mass_rotor]])
        self.C_h = np.array([[1.0,0.0]])

        self.Wn_h = 2.2/3.
        self.zeta_h = 1
        self.des_char_poly_h = [1,2*self.zeta_h*self.Wn_h,self.Wn_h**2]
        self.des_poles_h = np.roots(self.des_char_poly_h)

        # Latitudinal Constants
        self.A_l = np.array([[0.,0.,1.,0.],[0.,0.,0.,1.],[0.,-vtol.g,-vtol.dragCoef/(vtol.mass_centerpod+2*vtol.mass_rotor),0.],[0.,0.,0.,0.]])
        self.B_l = np.array([[0.],[0.],[0.],[1./(vtol.inertia_centerpod+2*vtol.mass_rotor*vtol.rod_length**2)]])
        self.C_l = np.array([[1.,0.,0.,0.],[0.,1.,0.,0.]])

        self.Wn_theta = 2.2/0.8
        self.Wn_Z = 2.2/3.
        self.zeta_theta = 0.707
        self.zeta_Z = 0.707
        self.des_char_poly_lat = np.convolve([1, 2*self.zeta_Z*self.Wn_Z, self.Wn_Z**2], [1, 2*self.zeta_theta*self.Wn_theta, self.Wn_theta**2])
        self.des_poles_lat = np.roots(self.des_char_poly_lat)

        # For longitudinal control
        if np.linalg.matrix_rank(ctl.ctrb(self.A_h,self.B_h)) != 2:
            print("System not controllable")
        else:
            self.K_h = (ctl.acker(self.A_h,self.B_h,self.des_poles_h))
            self.kr_h = -1.0/(self.C_h@np.linalg.inv(self.A_h-self.B_h@self.K_h)@self.B_h)

        # For latitudina control
        if np.linalg.matrix_rank(ctl.ctrb(self.A_l,self.B_l)) != 4:
            print("System not controllable")
        else:
            self.K_lat = ctl.acker(self.A_l,self.B_l,self.des_poles_lat)
            self.Cr = np.array([[1.0,0.0,0.0,0.0]])
            self.kr_lat = -1.0/(self.Cr@np.linalg.inv(self.A_l-self.B_l@self.K_lat)@self.B_l)

    def update(self,z_ref,height_ref,state):
        z = state.item(0)
        h = state.item(1)
        theta = state.item(2)
        z_ = state.item(3)
        h_ = state.item(4)
        theta_ = state.item(5)

        force_min = (vtol.mass_centerpod + 2*vtol.mass_rotor)*vtol.g
        force = force_min - self.K_h@np.array([[h],[h_]]) + self.kr_h*height_ref
        tau = -self.K_lat@np.array([[z],[theta],[z_],[theta_]]) + self.kr_lat*z_ref
        force = float(force)
        tau = float(tau)


        rotForceLeft = force/2 - tau/(2*vtol.rod_length)
        rotForceRight = force/2 + tau/(2*vtol.rod_length)

        rotForce = [rotForceLeft,rotForceRight]
        rotForce = self.saturate(rotForce)
        
        return rotForce

    def saturate(self,rotForce):

        if abs(rotForce[0]) > self.rotForce_limit:
            rotForce[0] = self.rotForce_limit
        if abs(rotForce[1]) > self.rotForce_limit:
            rotForce[1] = self.rotForce_limit

        return rotForce



