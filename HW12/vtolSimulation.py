import matplotlib.pyplot as plt
from math import pi
import sys 
sys.path.append('..')
from vtolDynamics import vtolDynamics
from HW2.vtolAnimation import vtolAnimation
from HW2.dataPlotter import dataPlotter
from HW2.signalGenerator import signalGenerator
from vtolController import vtolController
from numpy import *

class vtol():

    def __init__(self):
        self.dynamics = vtolDynamics()
        self.animate = vtolAnimation()
        self.control = vtolController()

system_1 = vtol()
plot = dataPlotter()

reference = signalGenerator(amplitude=1, frequency=0.05)
reference_2 = signalGenerator(amplitude=1, frequency=0.05)

t = 0.
height_ref = 3.
zee_ref = signalGenerator(amplitude = 1,frequency=0.05)
system_state = system_1.dynamics.state
rotForce = [0,0]
z_ref = 2

while t < 180:
    t_next = t+0.1
    while t < t_next:
        z_ref = zee_ref.square(t)
        height_ref = zee_ref.square(t)
        rotForce = system_1.control.update(z_ref,height_ref,system_state)
        system_state = system_1.dynamics.updated_state(rotForce)
        r = [reference.step(t),reference_2.step(t)]
        t += 0.01
    system_1.animate.update([system_state.item(0),system_state.item(1),system_state.item(2)])
    plot.update(t,r,array([system_state.item(0),system_state.item(1),system_state.item(2)]),rotForce[0])
    
    plt.pause(0.001)

plt.close()



