import vtolParam as P
import numpy as np
import control as ctl
from scipy import signal
import sys

# Observer Latitudinal

dist_obsv_pole_h = 0.0# Add something here

A_obs_h = np.concatenate((np.concatenate((P.A_obs_h,P.B_obs_h),axis=1),np.zeros((1,3))),axis=0)
B_obs_h = np.concatenate((P.B_obs_h,np.zeros((1,1))),axis=0)
C_obs_h = np.concatenate((P.C_obs_h,np.zeros((1,1))),axis=1)

des_char_est_h = np.array([1.,2.*P.zeta_h*P.Wn_obs_h,P.Wn_obs_th**2.])
des_poles_est_h = np.roots(des_char_est_h)
des_obs_poles_h = np.concatenate((des_poles_est_h,dist_obsv_pole_h*np.ones(1)))

if np.linalg.matrix_rank(ctl.ctrb(A_obs_h.T,C_obs_h.T)) != 3:
    print("System not observable")
else:
    L2_h = ctl.acker(A_obs_h.T,C_obs_h.T,des_obs_poles_h).T
    L_h = np.array([[L2_h.item(0)],[L2_h.item(1)]])
    Ld_h = L2_h.item(0)


# Observer Longitudinal

dist_obsv_pole_z = np.array([0])

A_obs_z = np.concatenate((np.concatenate((P.A_obs_z,P.B_obs_z),axis=1),np.zeros((1,5))),axis=0)
C_obs_z = np.concatenate((P.C_obs_z,np.zeros((2,1))),axis=1)

des_char_poly_obs_z = np.convolve(np.convolve([1,2*P.zeta_th*P.Wn_obs_th,P.Wn_obs_th**2],[1,2*P.zeta_z*P.Wn_obs_z,P.Wn_obs_z**2]),np.poly(dist_obsv_pole_z))
des_obs_poles_z = np.roots(des_char_poly_obs_z)

if np.linalg.matrix_rank(ctl.ctrb(A_obs_z.T,C_obs_z.T)) != 5:
    print("System not observable")
else:
    L2_z = signal.place_poles(A_obs_z.T,C_obs_z.T,des_obs_poles_z).gain_matrix.T
    L_z = L2_z[0:4,0:2]
    Ld_z = L2_z[4:5,0:2]


