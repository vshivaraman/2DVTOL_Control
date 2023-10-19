import numpy as np
import control as ctl
from vtolDynamics import vtolDynamics

vtol = vtolDynamics()

# W and zeta parameters

Wn_h = 2.2/2
Wn_th = 2.2/0.8
Wn_z = 2.2/1
zeta_h = 0.6
zeta_th = 0.5
zeta_z = 0.8
integrator_pole_h = np.array([0.25])
integrator_pole_z = np.array([0.27])

# Longitudinal

A_h = np.array([[0.0,1.0,0.0],[0.0,0.0,0.0],[-1.0,0.0,0.0]])
B_h = np.array([[0.0],[1.0/(vtol.mass_centerpod+2*vtol.mass_rotor)],[0.0]])

des_char_poly_h = np.convolve([1,2*zeta_h*Wn_h,Wn_h**2],np.poly(integrator_pole_h))
des_poles_h = np.roots(des_char_poly_h)

if np.linalg.matrix_rank(ctl.ctrb(A_h,B_h)) != 3:
    print("System not controllable")
else:
    K1_h = ctl.acker(A_h,B_h,des_poles_h)
    K_h = np.array([[K1_h.item(0),K1_h.item(1)]])
    ki_h = K1_h.item(2)

# Latitudinal

A_z = np.array([[0.0,0.0,1.0,0.0,0.0],[0.0,0.0,0.0,1.0,0.0],[0.0,-vtol.g,-vtol.dragCoef/(vtol.mass_centerpod+2*vtol.mass_rotor),0.0,0.0],[0.0,0.0,0.0,0.0,0.0],[-1.0,0.0,0.0,0.0,0.0]])
B_z = np.array([[0.0],[0.0],[0.0],[1.0/(vtol.inertia_centerpod+2*vtol.mass_rotor*vtol.rod_length)],[0.0]])


des_char_poly_z = np.convolve(np.convolve([1,2*zeta_th*Wn_th,Wn_th**2],[1,2*zeta_z*Wn_z,Wn_z**2]),np.poly(integrator_pole_z))
des_poles_z = np.roots(des_char_poly_z)

if np.linalg.matrix_rank(ctl.ctrb(A_z,B_z)) != 5:
    print("System not controllable")
else:
    K1_z = ctl.acker(A_z,B_z,des_poles_z)
    K_z = np.matrix([K1_z.item(0),K1_z.item(1),K1_z.item(2),K1_z.item(3)])
    ki_z = K1_z.item(4)



