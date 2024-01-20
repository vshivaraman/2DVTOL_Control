# This program contains an animation of a VTOL taking the parameters Zv, Zt, h and theta

# First Task: Make a class for the system (Coordinates and the VTOL)

# Middle Cube rotated by an angle.
# Two lines connecting two small ellipsoids passing through the cube inclined at an angle.
# Two Mass having shape of a small ellipsoid.

import matplotlib.pyplot as plt
from math import cos,sin,pi
import matplotlib.patches as mpat
import matplotlib as mpl
import numpy as np

class vtolAnimation:

    def __init__(self):
        self.switch = True
        self.handle = [0,0,0,0,0,0]
        self.fig,self.ax = plt.subplots()
        self.ax.set_xlim(-5,5)
        self.ax.set_ylim(-2,3)
        self.ax.set_title('VTOL Animation')
        self.ax.set_xlabel('Z')
        self.ax.set_ylabel('H')

    def update(self,param):
        Z,h,theta = param[0],param[1],param[2] # Theta is in radians
        self.draw_centerPod(Z,h,theta)
        self.draw_rotor(Z,h,theta)
        self.draw_chassis(Z,h,theta)
        self.draw_target()
        self.ax.axhline(linestyle = '--',color = 'red')
        #self.ax.autoscale()
        
        if self.switch == True:
            self.switch = False

    def draw_centerPod(self,Z,h,theta):
        a = 0.100
        xy = (Z-a*sin(pi/4-theta)/(2**(1/2)),h-a*cos(pi/4-(theta))/(2**(1/2)))

        if self.switch == True:
            self.handle[0] = mpat.Rectangle((xy),a,a,theta*180/pi)
            self.ax.add_patch(self.handle[0])
        else:
            self.handle[0].set_xy((xy))
            self.handle[0].set_angle(theta*180/pi)


    def draw_rotor(self,Z,h,theta):
        d = 0.28
        center_1 = (Z+d*cos(theta),h+d*sin(theta))
        center_2 = (Z-d*cos(theta),h-d*sin(theta))

        if self.switch == True:
            self.handle[1] = mpat.Ellipse(center_1,0.15,0.05,theta*180/pi)
            self.handle[2] = mpat.Ellipse(center_2,0.15,0.05,theta*180/pi)
            self.ax.add_patch(self.handle[1])
            self.ax.add_patch(self.handle[2])
        else:
            self.handle[1].set_center(center_1)
            self.handle[1].set_angle(theta*180/pi)
            self.handle[2].set_center(center_2)
            self.handle[2].set_angle(theta*180/pi)
    
    def draw_chassis(self,Z,h,theta):
        a = 0.100
        d = 0.28-0.075
        X_1 = [Z+d*cos(theta),Z]
        X_2 = [Z-d*cos(theta),Z]

        Y_1 = [h+d*sin(theta),h]
        Y_2 = [h-d*sin(theta),h]
        if self.switch == True:
            self.handle[3], = self.ax.plot(X_1,Y_1,lw = 1,c = 'black') 
            self.handle[4], = self.ax.plot(X_2,Y_2,lw = 1,c = 'black')
        else:
            self.handle[3].set_xdata(X_1)
            self.handle[3].set_ydata(Y_1)
            self.handle[4].set_xdata(X_2)
            self.handle[4].set_ydata(Y_2)

    def draw_target(self):
        Zt = 1
        a = 0.08
        if self.switch == True:
            self.handle[5] = mpat.Rectangle((Zt,0),a,a)
            self.ax.add_patch(self.handle[5])
        else:
            pass
        
#vtol_1 = vtolAnimation()
#vtol_1.update([0,0,pi/90])
#plt.show()




