import numpy as np
from propeller import Propeller
from motor import Motor


# Mass budget inputs
m_p = 0.64  # Mass of the payload
m_3d = 1.2  # Mass of the 3D modelling system
m_pos = 0.4  # Mass of the positioning system
m_data = 0.02  # Mass of the data handling system
m_rad = 0.05  # Mass of the radio system
m_elec = 0.1  # Mass of the electrical components

# Mass of Hydrogen propulsion subsystem
m_fc = 3  # Mass of the hydrogen fuel cell
m_pr = 0.305  # Mass of the pressure regulator


class Hydrogen:
    pass


class Drone:
    def __init__(self, T, Nm, TW_R, Coaxial, Propeller, Motor):
        # General characteristics
        self.T = T  # hrs of flight time
        self.Nm = Nm  # Number of motors
        self.TW_R = TW_R  # Thrust to Weight Ratio
        self.Coaxial = Coaxial  # Coaxial or conventional propeller setup

        self.motor = Motor
        self.propeller = Propeller

        self.m_prop = Propeller.mass * Nm  # Mass of the propellers
        self.m_motor = Motor.mass * Nm  # Mass of the motors

    def compute_weight(self) -> None:
        m_tot = (
            m_p
            + self.m_prop
            + self.m_motor
            + m_3d
            + m_pos
            + m_data
            + m_elec
            + m_fc
            + m_pr
        )
        T_req = m_tot * 9.80665
        T_req_m = T_req / self.Nm
        N = self.propeller.required_rpm(T_req_m)
        M = self.propeller.forces(N)
        IV = self.motor.VandI(M, N)
        P = IV[0] * IV[1]
        P_tot = P * self.Nm
        E_tot = P_tot * self.T
