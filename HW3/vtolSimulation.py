import matplotlib.pyplot as plt
from math import pi
import sys 
sys.path.append('..')
from vtolDynamics import vtolDynamics
from HW2.vtolAnimation import vtolAnimation
from HW2.dataPlotter import dataPlotter
from HW2.signalGenerator import signalGenerator
from numpy import *

class vtol():

    def __init__(self):
        self.dynamics = vtolDynamics()
        self.animate = vtolAnimation()
        #self.control = vtolController()

system_1 = vtol()
plot = dataPlotter()

reference = signalGenerator(amplitude=1, frequency=0.1)
#force = signalGenerator(amplitude=1, frequency=1)

t = 0.
height_ref = 1.
system_state = system_1.dynamics.state
rotForce = [10.2,10.2]

while t < 180:
    t_next = t+0.1
    while t < t_next:
        if system_state.item(1) > 1.0:
            print(t)
        system_state = system_1.dynamics.updated_state(rotForce)
        r = [1,reference.step(t)]
        t += 0.01
    system_1.animate.update([system_state.item(0),system_state.item(1),system_state.item(2)])
    plot.update(t,r,array([system_state.item(0),system_state.item(1),system_state.item(2)]),rotForce[0])
    
    plt.pause(0.0001)

plt.close()
