import numpy as np


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














