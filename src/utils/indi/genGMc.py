#!/usr/bin/env python

import numpy as np
from scipy.constants import g as GRAVITY

def pendulumInertiaFromPeriod(P, R, m):
    # assuming a phyiscal pendulum with centroid R from the rotation point and
    # mass m. If it oscillates with period P, we can readily calculate the 
    # inertia I.
    # However, if R is rather large (m*R*R > I), then accuracy of this
    # calculation decreases, especially P and R must be known very precisely 
    # (+-1%) for a reasonable estimate (+- 10%)

    # https://phys.libretexts.org/Bookshelves/University_Physics/Book%3A_University_Physics_(OpenStax)/Book%3A_University_Physics_I_-_Mechanics_Sound_Oscillations_and_Waves_(OpenStax)/15%3A_Oscillations/15.05%3A_Pendulums
    # T = 2pi * sqrt(I/(mgR)) where R is distance axle to CoG
    # I = T^2 / (4pi^2) * mgR

    # inertia around rotation point
    Iaxle = (m*GRAVITY*R*P*P) / (4*np.pi*np.pi)

    # subtract parallel axis term, as Iaxle = I + m*R*R
    return Iaxle - m*R*R

#%% properties

# static properties
m = 0.3826 # kg
width = 0.127
length = 0.091
diagonal = np.hypot(width, length)

# inertia measurements
# let quad oscillate as a pendulum around the motor axles in all 3 directions
# record oscillation periods in seconds using the IMU
# MAKE SURE THAT COG IS EXACTLY IN THE CENTER
Raxle = 5 * 0.5e-3 # motor axle radius (to calculate location of rotation point)
Px = 0.5801
Py = 0.5709
Pz = 0.6295 # Period for z-axis rotation

# propeller config
direc = [1, -1, 1, -1] # motor rotation directions (positive -> right hand down), sequence FL, FR, RR, RL

# prop inertia
#Iprop = 1e-6 # shameless guess for now
Pp = (525 - 275) / 30 / 20 # counting frames for 20 periods, fps 30.000. One frame error = 5% error in inertia...
Rp = 36.0 * 1e-3 # 1mm error: 10% error in inertia --> estimated precision 0.5mm, so 5% error
mp = 1.40 * 1e-3 # 1% error: 1% error in inertia --> estimated precision 0.02g, so 1.5% error
Iprop = pendulumInertiaFromPeriod(Pp, Rp, mp) # red prop

# propeller performance
tau = 0.02 # spinup/spindown time constant
k = 2.58e-7 # constant in T = k omega^2 -- red 2.1inch pitch prop
Tmax = 4.2 # max thrust red prop
#k = 3.62e-7 # black 3inch pitch prop
#Tmax = 4.5 # max thrust black prop
CM = 0.01 # steady-state moment coefficient M = CM * T

# ESC+motor+prop performance at around 60% charge
k_ESC = 0.55 # nonlinearity of non-dimensional input to non-dimensional thrust according to T / Tmax = ku^2 + (1-k)u

#%% calc inertias
Rx = width/2 - Raxle
Ry = length/2 - Raxle
Rz = diagonal/2 - Raxle # distance for z-axis rotation
Ixx = pendulumInertiaFromPeriod(Px, Rx, m)
Iyy = pendulumInertiaFromPeriod(Py, Ry, m)
Izz = pendulumInertiaFromPeriod(Pz, Rz, m)

# 1% error in P: 7% error in inertia --> estimated precision 0.5ms, so bueno
# 1% error in R: 1.5% error in inertia --> estimated precision 5mm, so 8%...
# 1% error in m: 1% error in inertia --> estimated precision 0.1g, so bueno

#%% calc G
# let O = (Fx Fy Fz Mx My Mz)
# idea: DeltaO = B1 * DeltaT  +  B2 * DeltaWdot
# 
# where B1 holds information about thrust axes and motor locations
# and   B2 holds information about thrust axes and propeller inertia
# 
# using w = sqrt(T/k), first-order dynamics wdot = (w - w0)/tau and taylor 
# expansion of the square root results in:
# 
# DeltaO = B1 DeltaT  +  B2 / (2*w0*tau*k) * (DeltaT - DeltaTprev)
#
# Introduce specific generalized forces A = (fx fy fz taux tauy tauz) and
# normalized control U = T / Tmax
# DeltaA = G1 DeltaU  +  G2 / (2*w0*tau*k) * (DeltaU - DeltaUprev)
#    where  G1   == Minv * Tmax * B1  and  G2 == Minv * Tmax * B2
#    and    Minv == inv(diag(m,m,m,Ixx,Iyy,Izz))
# 
# this can later be inverted to compute DeltaT as:
# DeltaA + G2n / w0 DeltaUprev = ( G1 + G2 / w0 )  DeltaU
#    where G2n = G2 / (2*tau*k)
#
# or, assuming wdot feedback is available
# DeltaA + G2 * wdot_prev = ( G1 + G2n / w0 ) DeltaU

# calculate positions first, following FL, FR, RR, RL
X = .5 * np.array([
    [+length, -width, 0.],
    [+length, +width, 0.],
    [-length, +width, 0.],
    [-length, -width, 0.]
]).T
axes = [0., 0., -1.] # normlized
axes = np.reshape(np.repeat(axes, 4), (3, 4)) # normalized

B1 = np.zeros((6, 4))
B2 = np.zeros((6, 4))
B2[3:, :] = -np.array(direc)*axes * Iprop

# force components of G1
B1[:3, :] = axes

# moment components of G1
for i in range(4):
    # moment due to thrust axes offset from CoG
    B1[3:, i] = np.cross(X[:, i], axes[:, i])

    # moment due to rotor drag
    B1[3:, i] += -CM*direc[i]*axes[:, i] # negative because reaction force

M = np.diag([m,m,m,Ixx,Iyy,Izz]) # generalized mass
G1 = np.linalg.solve(M, B1) * Tmax
G2 = np.linalg.solve(M, B2) * Tmax
G2n = G2 / (2*tau*k)
w0_hover = np.sqrt(m*GRAVITY/4 / k)
Ginv_hover = np.linalg.pinv(G1 + G2n / w0_hover)

print(f"G1:\n{G1}")
print(f"G2:\n{G2}")
print(f"G2_normalizer: {1/(2*tau*k)}")
print(f"G2n:\n{G2n}")
print(f"inv(G1 + G2n/w0) at hover:\n{Ginv_hover}")
