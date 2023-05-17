import numpy as np


# Aerodynamic characteristics
rho = 1.225  # Air density
A = 5  # Aspect Ratio
epsilon = 0.85  # Oswald efficiency factor
labda = 0.75
zeta = 0.5
e = 0.83
Cfd = 0.015 # Zero lift drag Coefficient
alpha0 = 0
K0 = 6.11

class Propeller:

    def __init__(self, Dp, Hp, Bp, mass, N):
        self.Dp = Dp
        self.Hp = Hp
        self.Bp = Bp
        self.mass = mass
        self.N = N
        self.coefficients()
        self.forces()

    def coefficients(self):
        self.Ct = 0.25*np.pi**3*labda*zeta**2*self.Bp*K0*(epsilon*np.arctan2(self.Hp, np.pi*self.Dp)-alpha0)/(np.pi*A+K0)
        self.Cd = Cfd + np.pi*A*K0**2/e*(epsilon*np.arctan2(self.Hp, np.pi*self.Dp)-alpha0)**2/(np.pi*A+K0)**2
        self.Cm = 1/8/A*np.pi**2*self.Cd*zeta**2*labda*self.Bp**2

    def forces(self):
        self.T = self.Ct*rho*(self.N/60)**2*self.Dp**4
        self.M = self.Cm*rho*(self.N/60)**2*self.Dp**5