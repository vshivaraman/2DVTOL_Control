import matplotlib.pyplot as plt
import matplotlib.patches as mpat
import numpy as np
from math import cos,sin,pi

step = 0.01

class vtolDynamics():

    def __init__(self):
        self.state = np.array([[0.],[0.0],[0.*pi/180],[0.],[0.],[0.]])
        self.mass_centerpod = 1.5
        self.inertia_centerpod = 0.006
        self.mass_rotor = 0.27
        self.rod_length = 0.28
        self.dragCoef = 0.11
        self.g = 9.81
        self.rotForce_limit = 25

    def func(self,state,rotForce):
        Z = state.item(0)
        h = state.item(1)
        theta = state.item(2)
        Z_ = state.item(3)
        h_ = state.item(4)
        theta_ = state.item(5)
        rotForceLeft = rotForce[0]
        rotForceRight = rotForce[1]
        f1 = rotForceLeft*sin(theta)+rotForceRight*sin(theta)-self.dragCoef*Z_ # Should add drag force
        f2 = rotForceLeft*cos(theta)+rotForceRight*cos(theta) - (self.mass_centerpod+2*self.mass_rotor)*self.g # Should add the constant term (m1)
        f3 = (rotForceRight-rotForceLeft)*self.rod_length*0.5
        rhs = np.array([f1,f2,f3])
        
        m1 = self.mass_centerpod+2*self.mass_rotor
        m2 = self.mass_centerpod+2*self.mass_rotor
        m3 = self.inertia_centerpod+2*self.mass_rotor*self.rod_length**2
        lhs = np.array([[m1,0,0],[0,m2,0],[0,0,m3]])

        X__ = np.matmul(np.linalg.inv(lhs),rhs)
        return np.array([[Z_],[h_],[theta_],[X__.item(0)],[X__.item(1)],[X__.item(2)]])


    def rk4(self,state,rotForce):
        k1 = self.func(state,rotForce)
        k2 = self.func(state+step*k1/2,rotForce)
        k3 = self.func(state+step*k2/2,rotForce)
        k4 = self.func(state+step*k3,rotForce)
        self.state += 1/6*step*(k1+2*k2+2*k3+k4)

    def updated_state(self,rotForce):
        state = self.state
        """ Change the values in the list passed to the rk4 function
            for changing the force by the left and right rotor
            respectively"""
        self.saturate(rotForce,self.rotForce_limit)
        self.rk4(state,rotForce)
        return np.array([self.state])
    
    def saturate(self,rotForce,rotForce_limit):
        if abs(rotForce[0]) > rotForce_limit:
            rotForce[0] = rotForce_limit*np.sign(rotForce[0])
        if abs(rotForce[1]) > rotForce_limit:
            rotForce[1] = rotForce_limit*np.sign(rotForce[1])
            

#vtol = vtolDynamics()
#state = vtol.updated_state()
#print(state)

       


        
