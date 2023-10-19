import matplotlib.pyplot as plt
from numpy import *
from signalGenerator import signalGenerator
from vtolAnimation import vtolAnimation
from dataPlotter import dataPlotter
from math import pi

reference = signalGenerator(amplitude=3,frequency=0.1)
thetaRef = signalGenerator(amplitude=5*pi,frequency=10)
tauRef = signalGenerator(amplitude=5,frequency=0.1)

animation = vtolAnimation()
plot = dataPlotter()

t = 0

while t < 50:
    h = reference.square(t)
    theta = 180*thetaRef.sin(t)/pi
    z = tauRef.sawtooth(t)
    r = reference.square(t)
    u = 1

    state = [z,h,theta/180]
    animation.update(state)
    plot.update(t,r,array(state),u)

    t = t+0.01
    plt.pause(0.01)


