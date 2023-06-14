from typing import Optional

import numpy as np
import pandas as pd
from sklearn.metrics import mean_squared_error
import matplotlib as mpl

from src.components import I_components, g, mass_components, power_components
from src.components import c1, c2, c3, c4, c5
from src.propeller import Propeller
from src.motor import Motor
from src.power import HydrogenTank, FuelCell
from src.esc import ESC
from src.configuration import configuration

import matplotlib.pyplot as plt


mpl.rcParams['axes.prop_cycle'] = mpl.cycler(color=[c1, c2, c3, c4, c5])


class Drone:
    def __init__(
        self,
        config: Optional[dict] = None,
        propeller: Optional[Propeller] = None,
        motor: Optional[Motor] = None,
        esc: Optional[ESC] = None,
        tank_mass: Optional[float] = None,
        mh2: Optional[float] = None
    ) -> None:
        if not config:
            config = configuration.copy()

        # General characteristics: hrs of flight, number of motors, T/W ratio
        self.T, self.Nm, self.TW_R = config["mission"].values()

        if propeller:
            self.propeller = propeller
            config["propeller"] = propeller.dict
        else:
            self.propeller = Propeller(
                *config["propeller"].values()
            )

        if motor:
            self.motor = motor
            config["motor"] = motor.dict
        else:
            self.motor = Motor(*config["motor"].values())

        if esc:
            self.esc = esc
            config["esc"] = esc.dict
        else:
            self.esc = ESC(*config["esc"].values())

        self.fuelcell = FuelCell()

        self._config = config
        self.mass = self.compute_weight(tank_mass=tank_mass, mh2=mh2)

    def compute_weight(
        self, max_iter: int = 1000, tw_f=1.2, co_eff=0.9, tank_mass=None, mh2=None
    ) -> Optional[float]:
        m_prop = self.propeller.mass * self.Nm  # Mass of the propellers
        m_motor = self.motor.mass * self.Nm  # Mass of the motors
        m_esc = self.esc.mass * self.Nm
        if tank_mass is None and mh2 is None:
            m_tot = (
                mass_components
                + m_prop
                + m_motor
                + m_esc
            )
            diff = 10000
            i = 0
            while diff > 0.001:
                T_req = m_tot * g
                T_req_m = T_req / self.Nm / co_eff
                self.N = self.propeller.required_rpm(T_req_m)
                M = self.propeller.forces(self.N)[1]
                V, I = self.motor.VandI(M, self.N)
                P = V * I
                self.P_tot = P * self.Nm * tw_f + power_components
                E_tot = self.P_tot * self.T
                self.hyd = HydrogenTank(E_tot, tank_mass=tank_mass)
                m_hyd = self.hyd.tot_mass()
                m_new = (
                    mass_components
                    + m_prop
                    + m_motor
                    + m_esc
                    + m_hyd
                )
                diff = abs(m_new - m_tot)
                m_tot = m_new
                i += 1
                if i == max_iter:
                    return None
        else:
            m_tot = (
                    mass_components
                    + m_prop
                    + m_motor
                    + m_esc
                    + mh2
                    + tank_mass
            )
            T_req = m_tot * g
            T_req_m = T_req / self.Nm / co_eff
            self.N = self.propeller.required_rpm(T_req_m)
            M = self.propeller.forces(self.N)[1]
            V, I = self.motor.VandI(M, self.N)
            P = V * I
            self.P_tot = P * self.Nm * tw_f + power_components
            E_tot = self.P_tot * self.T
            self.hyd = HydrogenTank(E_tot, tank_mass=tank_mass, mh2=mh2)

        self.I_ratio = self.check_for_max(m_tot, co_eff)

        return m_tot

    def check_for_max(self, m_tot, co_eff):
        T_req = m_tot * g * self.TW_R
        T_req_m = T_req / self.Nm / co_eff
        N = self.propeller.required_rpm(T_req_m)
        M = self.propeller.forces(N)[1]
        V, I = self.motor.VandI(M, N)

        return I / self.motor.Immax

    def compute_endurance(self, av_t, co_eff=0.9, tw_f=1.2, mh2=0.12, Ppay=None, Ptot=False):
        if mh2 is None:
            E = self.hyd.mh2 * self.hyd.U
            T_req_m = self.mass * g / self.Nm / co_eff * av_t
        else:
            E = mh2 * self.hyd.U
            T_req_m = (self.mass - self.hyd.mh2 + mh2) * g / self.Nm / co_eff * av_t
        N = self.propeller.required_rpm(T_req_m)
        M = self.propeller.forces(N)[1]
        V, I = self.motor.VandI(M, N)
        P = V * I
        if Ppay is not None:
            P_tot = P * self.Nm * tw_f + Ppay
        else:
            P_tot = P * self.Nm * tw_f + power_components
        if Ptot:
            return P_tot
        else:
            return E / P_tot

    def plot_endurance_TW(self):
        avt_arr = np.arange(1, 2.05, 0.05)
        end_arr = []
        for avt in avt_arr:
            end_arr.append(self.compute_endurance(avt))
        plt.plot(avt_arr, end_arr, color=c3, linewidth=2)
        plt.grid()
        plt.xlabel("T/W Ratio [-]")
        plt.ylabel("Endurance [h]")
        plt.show()

    @staticmethod
    def plot(
        ax,
        x,
        y,
        xlabel,
        ylabel,
        xlimit=None,
        ylimit=None,
        label=None,
        title=None,
        verts=None,
        vertlabels=None,
    ):
        plt.grid(which="minor", linewidth=0.2)
        plt.grid(which="major", linewidth=1)
        plt.minorticks_on()

        if title:
            plt.title(title)

        if verts:
            for i, vert in enumerate(verts):
                if vertlabels:
                    plt.plot([vert, vert], [0, max(y) * 1.1], label=vertlabels[i], linestyle='dashed', linewidth=1.5)
                    ax.legend()
                else:
                    plt.plot([vert, vert], [0, max(y) * 1.1], linestyle='dashed', linewidth=1.5)

        if xlimit:
            plt.xlim(xlimit)
        if ylimit:
            plt.ylim(bottom=0)

            x = np.array(x)
            y = np.array(y)

            y_ = y * ((xlimit[0] < x) & (x < xlimit[1]))

            plt.ylim(top=max(y_) * 1.1)

        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.plot(x, y, alpha=1.0, label=label)

    def plot_PT(self):
        T_motor = []
        P_motor = []
        V_motor = []
        I_motor = []
        M_motor = []
        eta_motor = []

        throttle = []
        V_esc = []
        I_esc = []

        V_fc = []
        I_fc = []
        P_fc = []


        N_arr = range(0, 100000, 10)
        found = False

        for N in N_arr:
            T, M = self.propeller.forces(N)
            V, I = self.motor.VandI(M, N)
            P = I * V
            if self.motor.Immax < I and found is False:
                T3 = T
                N3 = N
                # print(T3*self.Nm/self.mass/g)
                found = True
            T_motor.append(T)
            P_motor.append(P)
            I_motor.append(I)
            V_motor.append(V)
            M_motor.append(M)

            eta_motor.append(M * N * 2 * np.pi / 60 / P)

            P_tot = self.Nm * P + power_components

            I_f, V_f = self.fuelcell.get_current_voltage(P_tot)

            I_fc.append(I_f)
            V_fc.append(V_f)

            P_fc.append(I_f * V_f)

            throt = self.esc.throttle(V, I, V_f)
            throttle.append(throt)

            I_e = self.esc.inputI(V, I, V_f)
            V_e = self.esc.inputV(V_f, I_f, 0.1)

            I_esc.append(I_e)
            V_esc.append(V_e)


        T1 = self.mass * g / self.Nm
        T2 = self.mass * 2 * g / self.Nm
        N1 = self.propeller.required_rpm(T1)
        N2 = self.propeller.required_rpm(T2)

        fig = plt.figure(figsize=[12, 6])
        # fig.suptitle("Performance plots for each engine", fontsize=20)

        ax1 = fig.add_subplot(2, 4, 1)
        Drone.plot(
            ax1,
            T_motor,
            P_motor,
            "Thrust [N]",
            "Power [W]",
            verts=[T1, T2, T3],
            vertlabels=["Hover", "Max Thrust", "Max Current"],
            xlimit=[0, T3 * 1.1],
            ylimit=[0, T3 * 1.1],
        )

        ax2 = fig.add_subplot(2, 4, 2)
        Drone.plot(
            ax2,
            N_arr,
            P_motor,
            "RPM [-]",
            "Power [W]",
            verts=[N1, N2, N3],
            xlimit=[0, N3 * 1.1],
            ylimit=[0, T3 * 1.1],
        )

        ax3 = fig.add_subplot(2, 4, 3)
        Drone.plot(
            ax3,
            N_arr,
            T_motor,
            "RPM [-]",
            "Thrust [N]",
            verts=[N1, N2, N3],
            xlimit=[0, N3 * 1.1],
            ylimit=[0, T3 * 1.1],
        )

        ax4 = fig.add_subplot(2, 4, 4)
        Drone.plot(
            ax4,
            N_arr,
            eta_motor,
            "RPM [-]",
            "Efficiency [-]",
            verts=[N1, N2, N3],
            xlimit=[0, N3 * 1.1],
            ylimit=[0, T3 * 1.1],
        )

        ax5 = fig.add_subplot(2, 4, 5)
        Drone.plot(
            ax5,
            N_arr,
            M_motor,
            "RPM [-]",
            "Torque [Nm]",
            verts=[N1, N2, N3],
            xlimit=[0, N3 * 1.1],
            ylimit=[0, T3 * 1.1],
        )

        ax6 = fig.add_subplot(2, 4, 6)
        Drone.plot(
            ax6,
            T_motor,
            M_motor,
            "Thrust [N]",
            "Torque [Nm]",
            verts=[T1, T2, T3],
            xlimit=[0, T3 * 1.1],
            ylimit=[0, T3 * 1.1],
        )

        ax7 = fig.add_subplot(2, 4, 7)
        Drone.plot(
            ax7,
            N_arr,
            I_motor,
            "RPM [-]",
            "Current [A]",
            verts=[N1, N2, N3],
            xlimit=[0, N3 * 1.1],
            ylimit=[0, T3 * 1.1],
        )

        ax8 = fig.add_subplot(2, 4, 8)
        Drone.plot(
            ax8,
            N_arr,
            V_motor,
            "RPM [-]",
            "Voltage [V]",
            verts=[N1, N2, N3],
            xlimit=[0, N3 * 1.1],
            ylimit=[0, T3 * 1.1],
        )

        plt.tight_layout()
        plt.show()

    def plot_ESC_FC(self):
        throttle = []
        V_esc = []
        I_esc = []

        V_fc = []
        I_fc = []
        P_fc = []

        N_arr = range(0, 10000, 10)
        found = False

        P_max = self.fuelcell.Imax * self.fuelcell.get_voltage(self.fuelcell.Imax)

        for N in N_arr:
            T, M = self.propeller.forces(N)
            V, I = self.motor.VandI(M, N)
            P = I * V

            V_f = self.fuelcell.get_current_voltage(self.Nm * P)[1]

            throt = self.esc.throttle(V, I, V_f)
            throttle.append(throt)

            I_e = self.esc.inputI(V, I, V_f)
            I_f = I_e * self.Nm + I_components
            V_e = self.esc.inputV(V_f, I_f, 0.1)

            I_esc.append(I_e)
            V_esc.append(V_e)

            V_fc.append(V_f)
            I_fc.append(I_f)

            P_f = V_f * I_f
            P_fc.append(P_f)

            if P_f > P_max and found is False:
                T3 = T
                N3 = N
                print(T3 * self.Nm / self.mass / g)
                found = True

        T1 = self.mass * g / self.Nm
        T2 = self.mass * 2 * g / self.Nm
        N1 = self.propeller.required_rpm(T1)
        N2 = self.propeller.required_rpm(T2)

        fig = plt.figure(figsize=[10, 6])

        ax9 = fig.add_subplot(2, 3, 1)
        Drone.plot(
            ax9,
            N_arr,
            throttle,
            "RPM [-]",
            "Throttle [-]",
            verts=[N1, N2, N3],
            xlimit=[0, N3 * 1.1],
            ylimit=[0, T3 * 1.1],
            vertlabels=["Hover", "Max Thrust", "Max Power"]
        )

        ax10 = fig.add_subplot(2, 3, 2)
        Drone.plot(
            ax10,
            N_arr,
            V_esc,
            "RPM [-]",
            "Voltage ESC [V]",
            verts=[N1, N2, N3],
            xlimit=[0, N3 * 1.1],
            ylimit=[0, T3 * 1.1],
        )

        ax11 = fig.add_subplot(2, 3, 3)
        Drone.plot(
            ax11,
            N_arr,
            I_esc,
            "RPM [-]",
            "Current ESC [A]",
            verts=[N1, N2, N3],
            xlimit=[0, N3 * 1.1],
            ylimit=[0, T3 * 1.1],
        )

        ax12 = fig.add_subplot(2, 3, 4)
        Drone.plot(
            ax12,
            N_arr,
            V_fc,
            "RPM [-]",
            "Voltage Fuel cell [V]",
            verts=[N1, N2, N3],
            xlimit=[0, N3 * 1.1],
            ylimit=[0, T3 * 1.1],
        )

        ax13 = fig.add_subplot(2, 3, 5)
        Drone.plot(
            ax13,
            N_arr,
            I_fc,
            "RPM [-]",
            "Current Fuel cell [A]",
            verts=[N1, N2, N3],
            xlimit=[0, N3 * 1.1],
            ylimit=[0, T3 * 1.1],
        )

        ax14 = fig.add_subplot(2, 3, 6)
        Drone.plot(
            ax14,
            N_arr,
            P_fc,
            "RPM [-]",
            "Power Fuel cell [W]",
            verts=[N1, N2, N3],
            xlimit=[0, N3 * 1.1],
            ylimit=[0, T3 * 1.1],
        )

        plt.tight_layout()
        plt.show()


    def validation(self):
        try:
            data = pd.read_csv(f'../experimental_data/{self.propeller.name}.csv')
        except FileNotFoundError:
            return
        data = data.sort_values(by=['Rotation speed (rpm)'])
        N = data['Rotation speed (rpm)']
        T = data['Thrust (kgf)'] * 9.80665
        M = data['Torque (N⋅m)']
        P = data['Electrical power (W)']
        I = data['Current (A)']
        eta = data['Motor & ESC efficiency (%)'] / 100
        T_model = []
        M_model = []
        P_model = []
        eta_model = []
        for i in N:
            Ti, Mi = self.propeller.forces(i)
            V, Ii = self.motor.VandI(Mi, i)
            etai = Mi * i / (V * Ii) * 2 * np.pi / 60
            P_model.append(V * Ii)
            T_model.append(Ti)
            M_model.append(Mi)
            eta_model.append(etai)

        fig = plt.figure(figsize=[8, 6])

        ax1 = fig.add_subplot(2, 2, 1)
        ax1.plot(N, M, label='Actual Values', linestyle="dashed", alpha=0.8)
        ax1.plot(N, M_model, label='Model')
        plt.grid(which='major')
        plt.xlim(left=0)
        plt.ylim(bottom=0)

        plt.xlabel('RPM [-]')
        plt.ylabel('Torque [Nm]')
        plt.legend()

        ax2 = fig.add_subplot(2, 2, 2)
        ax2.plot(N, T, linestyle="dashed", alpha=0.8)
        ax2.plot(N, T_model)
        plt.xlabel('RPM [-]')
        plt.ylabel('Thrust [N]')
        plt.grid(which='major')
        plt.xlim(left=0)
        plt.ylim(bottom=0)

        ax3 = fig.add_subplot(2, 2, 3)
        ax3.plot(N, P, linestyle="dashed", alpha=0.8)
        ax3.plot(N, P_model)
        plt.xlabel('RPM [-]')
        plt.ylabel('Power [W]')
        plt.grid(which='major')
        plt.xlim(left=0)
        plt.ylim(bottom=0)

        ax4 = fig.add_subplot(2, 2, 4)
        ax4.plot(N, eta, linestyle="dashed", alpha=0.8)
        ax4.plot(N, eta_model)
        plt.xlabel('RPM [-]')
        plt.ylabel('Efficiency [-]')
        plt.grid(which='major')
        plt.xlim(left=0)
        plt.ylim(bottom=0)

        T_err = mean_squared_error(T, T_model, squared=False)
        M_err = mean_squared_error(M, M_model, squared=False)
        P_err = mean_squared_error(P, P_model, squared=False)
        eta_err = mean_squared_error(eta, eta_model, squared=False)

        T_mean = np.mean(T_model)
        M_mean = np.mean(M_model)
        P_mean = np.mean(P_model)
        eta_mean = np.mean(eta_model)

        print('Thrust Error', T_err)
        print('Moment Error', M_err)
        print('Power Error', P_err)
        print('Eff Error', eta_err)

        print(T_mean)
        print(M_mean)
        print(P_mean)
        print(eta_mean)

        print(T_err/T_mean)
        print(M_err/M_mean)
        print(P_err/P_mean)
        print(eta_err/eta_mean)

        plt.tight_layout()
        plt.show()

    def __repr__(self):
        return (
            f"{self.propeller} |  {self.motor} | {self.mass:.2f} kg | {self.N:.2f} rpm"
        )

    def __str__(self):
        return self.__repr__()

    @property
    def config(self):
        return self._config.copy()


if __name__ == "__main__":
    drone = Drone()
    drone.plot_PT()
