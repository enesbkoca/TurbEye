from typing import Optional

import numpy as np

from src.propeller import Propeller
from src.motor import Motor
from src.hydrogen import Hydrogen
from src.configuration import configuration

import matplotlib.pyplot as plt

g = 9.80665

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

P_cam = 10
P_ch = 5
P_3d = 15
P_pos = 1
P_data = 20
P_rad = 1
P_elec = 5

P_pay = P_cam + P_ch + P_3d + P_pos + P_data + P_rad + P_elec


class Drone:
    def __init__(
        self,
        config: Optional[dict] = None,
        propeller: Optional[Propeller] = None,
        motor: Optional[Motor] = None,
    ) -> None:
        if not config:
            config = configuration.copy()

        self._config = config

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

    def compute_weight(self, max_iter: int = 1000, tw_f = 1.2, co_eff = 0.9) -> Optional[float]:
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
            + m_pr
            + m_ch
            + m_rad
        )
        diff = 10000
        i = 0
        while diff > 0.001:
            T_req = m_tot * g
            T_req_m = T_req / self.Nm / co_eff
            self.N = self.propeller.required_rpm(T_req_m)
            M = self.propeller.forces(self.N)[1]
            IV = self.motor.VandI(M, self.N)
            P = IV[0] * IV[1]
            P_tot = P * self.Nm * tw_f + P_pay
            E_tot = P_tot * self.T
            self.hyd = Hydrogen(E_tot)
            m_hyd = self.hyd.tot_mass()
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
                + m_rad
            )
            diff = abs(m_new - m_tot)
            m_tot = m_new
            i += 1
            if i == max_iter:
                return None

        self.I_ratio = self.check_for_max(m_tot, co_eff)
        if self.I_ratio > 1:
            return None

        return m_tot

    def check_for_max(self, m_tot, co_eff):
        T_req = m_tot * g * self.TW_R
        T_req_m = T_req / self.Nm / co_eff
        N = self.propeller.required_rpm(T_req_m)
        M = self.propeller.forces(N)[1]
        IV = self.motor.VandI(M, N)
        print(IV[0], self.motor.Immax)
        return IV[0]/self.motor.Immax

    def compute_endurance(self, av_t, co_eff=0.9, tw_f=1.2):
        E = self.hyd.mh2 * self.hyd.U
        T_req_m = self.mass * g / self.Nm / co_eff * av_t
        N = self.propeller.required_rpm(T_req_m)
        M = self.propeller.forces(N)[1]
        IV = self.motor.VandI(M, N)
        P = IV[0] * IV[1]
        P_tot = P * self.Nm * tw_f + P_pay
        return E/P_tot

    def plot_endurance_TW(self):
        avt_arr = np.arange(1, 2, 0.05)
        end_arr = []
        for avt in avt_arr:
            end_arr.append(self.compute_endurance(avt))
        plt.plot(avt_arr, end_arr)
        plt.title('Endurance - T/W Ratio Curve')
        plt.xlabel('T/W Ratio [-]')
        plt.ylabel('Endurance [h]')
        plt.show()

    @staticmethod
    def plot(ax, x, y, xlabel, ylabel, label=None, title=None, verts=None, vertlabels=None):

        plt.grid(which='minor', linewidth=0.2)
        plt.grid(which='major', linewidth=1)
        plt.minorticks_on()

        if title:
            plt.title(title)

        if verts:
            for i, vert in enumerate(verts):
                if vertlabels:
                    plt.plot([vert, vert], [min(y), max(y)], label=vertlabels[i])
                    ax.legend()
                else:
                    plt.plot([vert, vert], [min(y), max(y)])

        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.plot(x, y, alpha=1.0, label=label)

    def plot_PT(self):
        T_arr = []
        P_arr = []
        eta_arr = []
        N_arr = range(0, 5000, 10)
        found = False
        for N in N_arr:
            T, M = self.propeller.forces(N)
            IV = self.motor.VandI(M, N)
            P = IV[0] * IV[1]
            if self.motor.Immax < IV[0] and found is False:
                T3 = T
                N3 = N
                found = True
            T_arr.append(T)
            P_arr.append(P)
            eta_arr.append(M*N*2*np.pi/60/P)

        T1 = self.mass*g/self.Nm
        T2 = self.mass*2*g/self.Nm
        N1 = self.propeller.required_rpm(T1)
        N2 = self.propeller.required_rpm(T2)

        fig = plt.figure(figsize=[10, 6])
        fig.suptitle('Performance plots for each engine', fontsize=20)
        ax1 = fig.add_subplot(2, 2, 1)
        Drone.plot(ax1, T_arr, P_arr, 'Thrust [N]', 'Power [W]', [0, 101, 20, 4], [0, 3001, 500, 100],
                  verts=[T1, T2 ,T3], vertlabels=['Hover', 'Max Thrust', 'Max Current'])

        ax2 = fig.add_subplot(2, 2, 2)
        Drone.plot(ax2, N_arr, P_arr, 'RPM [-]', 'Power [W]', [0, 5001, 1000, 200], [0, 3001, 500, 100],
                  verts=[N1, N2, N3])

        ax3 = fig.add_subplot(2, 2, 3)
        Drone.plot(ax3, N_arr, T_arr, 'RPM [-]', 'Thrust [N]', [0, 5001, 1000, 200], [0, 101, 20, 4],
                  verts=[N1, N2, N3])

        ax4 = fig.add_subplot(2, 2, 4)
        Drone.plot(ax4, N_arr, eta_arr, 'RPM [-]', 'Efficiency [-]', [0, 5001, 1000, 200], [0, 1.001, 0.2, 0.04],
                  verts=[N1, N2, N3])

        plt.tight_layout()
        plt.show()

    def __repr__(self):
        return (
            f"{self.propeller} |  {self.motor} | {self.mass:.2f} kg | {self.N:.2f} rpm"
        )

    @property
    def config(self):
        return self._config.copy()


if __name__ == "__main__":
    drone = Drone()
    print(drone.mass)
