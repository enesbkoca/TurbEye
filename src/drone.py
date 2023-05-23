from propeller import Propeller
from motor import Motor
from hydrogen import Hydrogen
import json
from typing import Optional

# Mass budget inputs
m_p = 0.64  # Mass of the payload
m_3d = 1.2  # Mass of the 3D modelling system
m_pos = 0.4  # Mass of the positioning system
m_data = 0.02  # Mass of the data handling system
m_rad = 0.05  # Mass of the radio system
m_elec = 0.1  # Mass of the electrical components
m_ch = 0.5  # Mass of the chassis

# Mass of Hydrogen propulsion subsystem
m_fc = 3  # Mass of the hydrogen fuel cell
m_pr = 0.305  # Mass of the pressure regulator


class Drone:
    def __init__(
        self,
        config: Optional[dict] = None,
        propeller: Optional[Propeller] = None,
        motor: Optional[Motor] = None,
    ) -> None:
        if not config:
            with open("config.json", "r") as file:
                config = json.load(file)

        # General characteristics
        self.T = config["T"]  # hrs of flight time
        self.Nm = config["Nm"]  # Number of motors
        self.TW_R = config["TW_R"]  # Thrust to Weight Ratio

        if propeller:
            self.propeller = propeller
        else:
            self.propeller = Propeller(
                config["Dp"], config["Hp"], config["Bp"], config["m_prop"]
            )

        if motor:
            self.motor = motor
        else:
            self.motor = Motor(
                config["Kv0"],
                config["Um0"],
                config["Im0"],
                config["Rm"],
                config["Immax"],
                config["m_motor"],
            )

        self.mass = self.compute_weight()

    def compute_weight(self, max_iter: int = 1000) -> Optional[float]:
        m_prop = self.propeller.mass * self.Nm  # Mass of the propellers
        m_motor = self.motor.mass * self.Nm  # Mass of the motors
        m_tot = (
            m_p + m_prop + m_motor + m_3d + m_pos + m_data + m_elec + m_fc + m_pr + m_ch
        )
        diff = 10000
        i = 0
        while diff > 0.001:
            T_req = m_tot * 9.80665
            T_req_m = T_req / self.Nm * 2 / 1.8
            self.N = self.propeller.required_rpm(T_req_m)
            M = self.propeller.forces(self.N)[1]
            IV = self.motor.VandI(M, self.N)
            P = IV[0] * IV[1]
            P_tot = P * self.Nm
            E_tot = P_tot * self.T * 1.2
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
                + m_hyd
                + m_ch
            )
            diff = abs(m_new - m_tot)
            m_tot = m_new
            i += 1
            if i == max_iter:
                return None

        if IV[0] > self.motor.Immax:
            # print("Current greater than Immax", f"{self.propeller} |  {self.motor}")
            return None

        return m_tot

    def __repr__(self):
        return (
            f"{self.propeller} |  {self.motor} | {self.mass:.2f} kg | {self.N:.2f} rpm"
        )
