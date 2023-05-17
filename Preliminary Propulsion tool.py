import numpy as np


# Inputs

# # General characteristics
# T = 3  # hrs of flight time
# Nm = 8  # Number of motors
# TW_R = 2  # Thrust to Weight Ratio
# Coaxial = True  # Coaxial or conventional propeller setup
# Hydrogen = True  # Hydrogen or battery powered

# Mass budget inputs
m_p = 0.64  # Mass of the payload
m_3d = 1.2  # Mass of the 3D modelling system
m_pos = 0.4  # Mass of the positioning system
m_data = 0.02  # Mass of the data handling system
m_rad = 0.05  # Mass of the radio system
m_elec = 0.1  # Mass of the electrical components

# Mass of Hydrogen propulsion subsystem
m_fc = 3  # Mass of the hydrogen fuel cell
m_pr = 0.305  #  Mass of the pressure regulator


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

class Drone:

    def __init__(self, T, Nm, TW_R, Coaxial, Hydrogen, Propeller, Motor):
        # General characteristics
        self.T = T  # hrs of flight time
        self.Nm = Nm  # Number of motors
        self.TW_R = TW_R  # Thrust to Weight Ratio
        self.Coaxial = Coaxial  # Coaxial or conventional propeller setup
        self.Hydrogen = Hydrogen  # Hydrogen or battery powered

        # Motor characteristics
        self.Kv0 = Motor.Kv0  # KV value of the motor
        self.Um0 = Motor.Um0  # No load Voltage
        self.Im0 = Motor.Im0  # No load current
        self.Rm = Motor.Rm  # Motor resistance
        self.Immax = Motor.Immax  # Motor max current

        # Propeller characteristics
        self.Dp = Propeller.Dp  # Propeller diameter
        self.Hp = Propeller.Hp  # Propeller pitch
        self.Bp = Propeller.Bp  # Number of blades in propeller

        self.m_prop = Propeller.mass * Nm  # Mass of the propellers
        self.m_motor = Motor.mass * Nm  # Mass of the motors

class Motor:

    def __init__(self, Kv0, Um0, Im0, Rm, Immax, mass, Propeller):
        self.Kv0 = Kv0
        self.Um0 = Um0
        self.Im0 = Im0
        self.Rm = Rm
        self.Immax = Immax
        self.mass = mass
        self.propeller = Propeller

    def coefficients(self):
        M = self.Propeller.M
        self.Kt =

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











