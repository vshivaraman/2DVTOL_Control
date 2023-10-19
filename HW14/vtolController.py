import numpy as np
import control as ctl
from math import pi
from vtolDynamics import vtolDynamics
import vtolParam as P
import vtolParamHW14 as P14

vtol = vtolDynamics()
step = 0.01

class vtolController:
    
    def __init__(self):
        self.obs_state = np.array([[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]])
        self.rotForce_limit = 15
        self.integrator_h = 0.0
        self.integrator_z = 0.0
        self.force_dl = 0.0
        self.torque_dl = 0.0
        self.error_dl_h = 0.0
        self.error_dl_z = 0.0
        self.K_h = P.K_h
        self.ki_h = P.ki_h
        self.K_z = P.K_z
        self.ki_z = P.ki_z

        self.A_obs_h = P14.A_obs_h
        self.B_obs_h = P14.B_obs_h
        self.C_obs_h = P14.C_obs_h
        self.A_obs_z = P14.A_obs_z
        self.B_obs_z = P.B_z
        self.C_obs_z = P14.C_obs_z

        self.L_h = P14.L2_h
        self.Ld_h = P14.Ld_h
        self.L_z = P14.L2_z
        self.Ld_z = P14.Ld_z 

    def update(self,z_ref,height_ref,y):
        obs_state,d_hat = self.update_observer(y) # Let d_hat be a list
        z = obs_state.item(0)
        h = obs_state.item(1)
        theta = obs_state.item(2)
        z_ = obs_state.item(3)
        h_ = obs_state.item(4)
        theta_ = obs_state.item(5)

        force_min = (vtol.mass_centerpod + 2*vtol.mass_rotor)*vtol.g
        error_h = height_ref-h
        self.integrateError_h(error_h)
        force = force_min - self.K_h@np.array([[h],[h_]]) + self.ki_h*self.integrator_h - d_hat[0] # Change
        error = z_ref-z
        self.integrateError_z(error)
        tau = -self.K_z@np.array([[z],[theta],[z_],[theta_]]) + self.ki_z*self.integrator_z - d_hat[1]
        force = float(force)
        tau = float(tau)

        self.force_dl = force
        self.torque_dl = tau

        rotForceLeft = force/2 - tau/(2*vtol.rod_length)
        rotForceRight = force/2 + tau/(2*vtol.rod_length)

        rotForce = [rotForceLeft,rotForceRight]
        rotForce = self.saturate(rotForce)
        
        return rotForce,obs_state,d_hat

    def integrateError_h(self,error):
        self.integrator_h = self.integrator_h + (0.01/2.0)*(error+self.error_dl_h)
        self.error_dl_h = error

    def integrateError_z(self,error):
        self.integrator_z = self.integrator_z + (0.01/2.0)*(error+self.error_dl_z)
        self.error_dl_z = error

    def update_observer(self,y_m):
        #print(y_m)
        F1 = self.observer_func(self.obs_state,y_m)
        F2 = self.observer_func(self.obs_state+step/2*F1,y_m)
        F3 = self.observer_func(self.obs_state+step/2*F2,y_m)
        F4 = self.observer_func(self.obs_state+step*F3,y_m)
        self.obs_state += step/6*(F1+2*F2+2*F3+F4)
        #print(self.obs_state)
        
        x_hat = np.array([[self.obs_state.item(0)],[self.obs_state.item(1)],[self.obs_state.item(2)],[self.obs_state.item(3)],[self.obs_state.item(4)],[self.obs_state.item(5)]])
        d_hat = [self.obs_state.item(6),self.obs_state.item(7)]
        #print(d_hat)

        return x_hat,d_hat

    def observer_func(self,obs_state,y_m):
        #print(y_m)
        force = (vtol.mass_centerpod+2*vtol.mass_rotor)*vtol.g
        obs_h = obs_state.item(1)
        obs_h_ = obs_state.item(4)
        obs_state_h = np.array([[obs_h],[obs_h_],[obs_state.item(6)]]) # Here

        obs_z = obs_state.item(0)
        obs_theta = obs_state.item(2)
        obs_z_ = obs_state.item(3)
        obs_theta_ = obs_state.item(5)

        obs_state_z = np.array([[obs_z],[obs_theta],[obs_z_],[obs_theta_],[obs_state.item(7)]])

        y_m_h = np.array([[y_m.item(1)]])
        y_m_z = np.array([[y_m.item(0)],[y_m.item(2)]])
        
        X_h = self.A_obs_h@obs_state_h+self.B_obs_h*(self.force_dl-force)+self.L_h@(y_m_h-self.C_obs_h@obs_state_h)
        X_z = self.A_obs_z@obs_state_z+self.B_obs_z*self.torque_dl+self.L_z@(y_m_z-self.C_obs_z@obs_state_z)
        
        return np.array([[X_z.item(0)],[X_h.item(0)],[X_z.item(1)],[X_z.item(2)],[X_h.item(1)],[X_z.item(3)],[X_h.item(2)],[X_z.item(4)]]) # Return something 

    def saturate(self,rotForce):

        if abs(rotForce[0]) > self.rotForce_limit:
            rotForce[0] = self.rotForce_limit
        if abs(rotForce[1]) > self.rotForce_limit:
            rotForce[1] = self.rotForce_limit

        return rotForce



