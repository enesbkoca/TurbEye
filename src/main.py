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

    def __init__(self, E):
        self.U = 34000
        self.mh2 = E/self.U

    def tank_mass(self):
        return 41.187*self.mh2**2 + 8.38*self.mh2 + 1.0587

    def tot_mass(self):
        return self.mh2 + self.tank_mass()


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

    def compute_weight(self, max_iter=1000) -> None:
        m_tot = (
            m_p
            + self.m_prop
            + self.m_motor
            + m_3d
            + m_pos
            + m_data
            + m_elec
            + m_fc
            + m_pr)
        diff = 10000
        i = 0
        while diff > 0.001:
            T_req = m_tot * 9.80665
            T_req_m = T_req / self.Nm
            N = self.propeller.required_rpm(T_req_m)
            M = self.propeller.forces(N)
            IV = self.motor.VandI(M, N)
            P = IV[0] * IV[1]
            P_tot = P * self.Nm
            E_tot = P_tot * self.T
            hyd = Hydrogen(E_tot)
            m_hyd = hyd.tot_mass()
            m_new = (
                    m_p
                    + self.m_prop
                    + self.m_motor
                    + m_3d
                    + m_pos
                    + m_data
                    + m_elec
                    + m_fc
                    + m_pr
                    + m_hyd)
            diff = abs(m_new-m_tot)
            m_tot = m_new
            i += 1
            if i == max_iter:
                return None
        return m_tot


