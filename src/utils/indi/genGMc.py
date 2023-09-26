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


###################################
#%% properties
###################################

#%% inertia measurements

# let quad oscillate as a pendulum around the motor axles in all 3 directions
# record oscillation periods in seconds using the IMU
# MAKE SURE THAT COG IS EXACTLY IN THE CENTER
m = 0.3826 # kg
Px = 0.5801
Py = 0.5709
Pz = 0.6295 # Period for z-axis rotation
Raxle = 5 * 0.5e-3 # motor axle radius (to calculate location of rotation point)

# prop inertia measurements (measured for red 2.1inch prop)
Pp = (525 - 275) / 30 / 20 # counting frames for 20 periods, fps 30.000. One frame error = 5% error in inertia...
Rp = 36.0 * 1e-3 # 1mm error: 10% error in inertia --> estimated precision 0.5mm, so 5% error
mp = 1.40 * 1e-3 # 1% error: 1% error in inertia --> estimated precision 0.02g, so 1.5% error

# motor bell inertia
mb = 7.6e-3 # measured
Rb = 0.8 * (19e-3)/2 # radius of gyration, assumed 80% of outside radius


#%% propeller/ESC/motor performance at 4S battery (see prop.py)

tau = 0.015 # spinup/spindown time constant
k = 1.89e-7 # constant in T = k omega^2 -- red 2.1inch pitch prop
Tmax = 4.2 # max thrust red prop
#k = 2.66e-7 # black 3inch pitch prop
#Tmax = 4.5 # max thrust black prop
CM = 0.01 # steady-state moment coefficient M = CM * T

# ESC+motor+prop performance at around 60% charge
k_ESC = 0.55 # nonlinearity of non-dimensional input to non-dimensional thrust according to T / Tmax = ku^2 + (1-k)u


#%% configuration: position, thrust axis and direction.
# using betaflight numbering
#   ^
#   |      # don't ask...
# 4   2
#  \ / 
#  / \
# 3   1

# body coordinates
#
#   ^ x
#   | 
# z x--> y 

# dimensions from motor axle to motor axle
width = 127e-3
length = 91e-3
diagonal = np.hypot(width, length)

# configuration
N = 4
X = .5 * np.array([
    [-length, +width, 0.],
    [+length, +width, 0.],
    [-length, -width, 0.],
    [+length, -width, 0.]
]).T
axes = [0., 0., -1.] # normlized thrust axis 
axes = np.reshape(np.repeat(axes, N), (3, N)) # repeated N times

direc = [-1, 1, 1, -1] # motor rotation directions (positive -> right hand along prop thrust vector), sequence FL, FR, RR, RL


###################################
#%% perform calculations
###################################

#%% inertias
Rx = width/2 - Raxle
Ry = length/2 - Raxle
Rz = diagonal/2 - Raxle # distance for z-axis rotation
Ixx = pendulumInertiaFromPeriod(Px, Rx, m)
Iyy = pendulumInertiaFromPeriod(Py, Ry, m)
Izz = pendulumInertiaFromPeriod(Pz, Rz, m)
Iprop = pendulumInertiaFromPeriod(Pp, Rp, mp)  +  mb * Rb*Rb

# 1% error in P: 7% error in inertia --> estimated precision 0.5ms, so bueno
# 1% error in R: 1.5% error in inertia --> estimated precision 5mm, so 8%...
# 1% error in m: 1% error in inertia --> estimated precision 0.1g, so bueno


#%% matrices

# let O = (Fx Fy Fz Mx My Mz)
# idea: DeltaO = B1 * DeltaT  +  B2 * DeltaWdot
# 
# where B1 holds information about thrust axes and motor locations
# and   B2 holds information about thrust axes and propeller inertia
# 
# using w = sqrt(T/k), first-order dynamics wdot = (w - w0)/tau and taylor 
# expansion of the square root results in:
# 
#   DeltaO = B1 DeltaT  +  B2 / (2*w0*tau*k) * (DeltaT - DeltaTprev)
#
# Introduce specific generalized forces A = (fx fy fz taux tauy tauz) with 
# units (N/kg N/kg N/kg Nm/(kgm^2) Nm/(kgm^2) Nm/(kgm^2)) and
# the normalized unitless control U = T / Tmax
#
#   DeltaA = G1 DeltaU  +  G2 / (2*w0*tau*k) * (DeltaU - DeltaUprev)
#      where  G1   == Minv * Tmax * B1  and  G2 == Minv * Tmax * B2
#      and    Minv == inv(diag(m,m,m,Ixx,Iyy,Izz))  called generalized mass matrix
# 
# this can later be inverted to compute DeltaU by solving:
#
#   DeltaA + G2n / w0 DeltaU_prev = ( G1 + G2 / w0 )  DeltaU
#      where G2n = G2 / (2*tau*k)
#
# or, assuming wdot feedback is available
#
#   DeltaA + G2 / Tmax * wdot_prev = ( G1 + G2n / w0 ) DeltaU

B1 = np.zeros((6, N))
B2 = np.zeros((6, N))
B2[3:, :] = -np.array(direc)*axes * Iprop

# force components of G1
B1[:3, :] = axes

# moment components of G1
for i in range(N):
    # moment due to thrust axes offset from CoG
    B1[3:, i] = np.cross(X[:, i], axes[:, i])

    # moment due to rotor drag
    B1[3:, i] += -CM*direc[i]*axes[:, i] # negative because reaction force

M = np.diag([m,m,m,Ixx,Iyy,Izz]) # generalized mass
G1 = np.linalg.solve(M, B1) * Tmax
G2 = np.linalg.solve(M, B2) * Tmax
G2n = G2 / (2*tau*k)
w0_hover = np.sqrt(m*GRAVITY/N / k)
Ginv_hover = np.linalg.pinv(G1 + G2n / w0_hover)


#%% print
print(f"G1:\n{G1}")
print(f"G2:\n{G2}")
print(f"G2_normalizer: {1/(2*tau*k)}")
print(f"G2n:\n{G2n}")
print(f"inv(G1 + G2n/w0) at hover:\n{Ginv_hover}")
