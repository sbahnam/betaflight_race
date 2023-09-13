import numpy as np
from scipy.constants import g as GRAVITY

def pendulumInertiaFromPeriod(P, R, m):
    # assuming a phyiscal pendulum with centroid R from the rotation point and
    # mass m. If it oscillates with period P, we can readily calculate the 
    # inertia I.
    # However, if R is rather large (m*R*R > I), then accuracy of this
    # calculation decreases, especially P and R must be known very precisely 
    # (+-1%) for a reasonable estimate

    # https://phys.libretexts.org/Bookshelves/University_Physics/Book%3A_University_Physics_(OpenStax)/Book%3A_University_Physics_I_-_Mechanics_Sound_Oscillations_and_Waves_(OpenStax)/15%3A_Oscillations/15.05%3A_Pendulums
    # T = 2pi * sqrt(I/(mgR)) where R is distance axle to CoG
    # I = T^2 / (4pi^2) * mgR

    # inertia around rotation point
    Iaxle = (m*GRAVITY*R*P*P) / (4*np.pi*np.pi)

    # subtract parallel axis term, as Iaxle = I + m*R*R
    return Iaxle - m*R*R


#%% properties

# static properties
m = 0.37 # kg. Needs to be pretty precise for inertia calc.
width = 0.14
length = 0.10
diagonal = np.hypot(width, length)

# inertia measurements
# let quad oscillate as a pendulum around the motor axles in all 3 directions
# record oscillation periods in seconds using the IMU
# MAKE SURE THAT COG IS EXACTLY IN THE CENTER
Raxle = 4 * 0.5e-3 # motor axle radius (to calculate location of rotation point)
Px = 0.50
Py = 0.50
Pz = 0.64 # Period for z-axis rotation

# propeller/motor properties
tau = 0.02 # spinup/spindown time constant
k = 2.58e-7 # constant in T = k omega^2 -- red 2.1inch pitch prop
Tmax = 4.2 # max thrust red prop
#k = 3.62e-7 # black 3inch pitch prop
#Tmax = 4.5 # max thrust black prop
Iprop = 1e-6 # shameless guess for now
#wmax = 40000/60*2*np.pi # max rotaional speed in rad/s -- red prop
CM = 0.01 # steady-state moment coefficient
direc = [1, -1, 1, -1] # motor rotation directions (positive -> right hand down), sequence FL, FR, RR, RL

#%% calc inertias
Rx = width/2 - Raxle
Ry = length/2 - Raxle
Rz = diagonal/2 - Raxle # distance for z-axis rotation
Ixx = pendulumInertiaFromPeriod(Px, Rx, m)
Iyy = pendulumInertiaFromPeriod(Py, Ry, m)
Izz = pendulumInertiaFromPeriod(Pz, Rz, m)

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
# DeltaA = G1 DeltaU  +  G2 / w0 * (DeltaU - DeltaUprev)
#    where  G1 == Minv * Tmax * B1  and  G2 == Minv * Tmax * B2 / (2*tau*k)
#    and Minv = inv(diag(m,m,m,Ixx,Iyy,Izz))
# 
# this can later be inverted to compute DeltaT as:
# DeltaA + G2 / w0 DeltaUprev = ( G1 + G2 / w0 )  DeltaU

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
G2 = np.linalg.solve(M, B2) * Tmax / (Tmax*2*tau*k)
Ginv_hover = np.linalg.pinv(G1 + G2 / (np.sqrt(m*GRAVITY/4 / k)))

print(f"G1:\n{G1}")
print(f"G2:\n{G2}")
print(f"inv(G1+G2) at hover:\n{Ginv_hover}")
