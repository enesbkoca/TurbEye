import numpy as np
from propeller import Propeller
from motor import Motor
from hydrogen import Hydrogen

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


class Drone:

    def __init__(self, T, Nm, TW_R, Propeller, Motor):

        # General characteristics
        self.T = T  # hrs of flight time
        self.Nm = Nm  # Number of motors
        self.TW_R = TW_R  # Thrust to Weight Ratio

        self.motor = Motor
        self.propeller = Propeller

    def compute_weight(self, x, max_iter=1000) -> None:
        # x = [0.5, 0.2, 2, 0.034, 100, 10, 0.4, 0.27, 19, 0.128, 3, 8, 2]
        if x is not None:
            self.T = x[10]
            self.Nm = x[11]
            self.TW_R = x[12]
            self.propeller.Dp = x[0]
            self.propeller.Hp = x[1]
            self.propeller.Bp = x[2]
            self.propeller.mass = x[3]
            self.motor.Kv0 = x[4]
            self.motor.Um0 = x[5]
            self.motor.Im0 = x[6]
            self.motor.Rm = x[7]
            self.motor.Immax = x[8]
            self.motor.mass = x[9]
        m_prop = self.propeller.mass * self.Nm  # Mass of the propellers
        m_motor = self.motor.mass * self.Nm  # Mass of the motors
        m_tot = (
            m_p
            + m_prop
            + m_motor
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
                    + m_prop
                    + m_motor
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
        if IV[0] > self.motor.Immax:
            print('Current greater than Immax')
        return m_tot


