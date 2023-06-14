import numpy as np
import matplotlib.pyplot as plt
from src.components import *


class HydrogenTank:
    def __init__(self, E, tank_mass=None, mh2=None):
        self.U = 34000
        if mh2 is None:
            self.mh2 = E / self.U
        else:
            self.mh2 = mh2
        self.tm = tank_mass

    def tank_mass(self):
        if self.tm is not None:
            return self.tm
        else:
            return 41.187 * self.mh2**2 + 8.38 * self.mh2 + 1.0587

    def tot_mass(self):
        return self.mh2 + self.tank_mass()


class FuelCell:
    def __init__(self):
        self.Imax = 75

    def get_voltage(self, I):
        def left_linear(x):
            return 53 - 1.4151 * x

        def right_linear(x):
            return 46.4505 - 0.17934 * x

        if I > self.Imax:
            raise ValueError(f"Current larger than maximum: {I}")
        elif 0 <= I <= 5.3:
            return left_linear(I)
        elif 5.3 < I <= self.Imax:
            return right_linear(I)
        else:
            raise ValueError(f"Cannot get voltage from current: {I}")

    def get_current_voltage(self, power):

        diff = 0.1

        left_I = 0
        right_I = self.Imax

        mid_I = (left_I + right_I) / 2
        mid_V = self.get_voltage(mid_I)

        mid_power = mid_I * mid_V

        while mid_I < self.Imax and abs(mid_power - power) > diff:

            if mid_power > power:
                right_I = mid_I
            elif mid_power < power:
                left_I = mid_I

            mid_I = (left_I + right_I) / 2
            mid_V = self.get_voltage(mid_I)

            mid_power = mid_I * mid_V

        return mid_I, mid_V

    def plot(self):
        I = np.arange(0, self.Imax, 0.1)
        get_voltage = np.vectorize(lambda x: self.get_voltage(x))
        V = get_voltage(I)
        P = I * V

        fig, ax1 = plt.subplots(figsize=(6,3))

        plt.grid(which="minor", axis="both", linewidth=0.2)
        plt.grid(which="major", axis="both", linewidth=1)
        plt.minorticks_on()

        ax2 = ax1.twinx()
        ax1.plot(I, V, color=c2)

        ax2.plot(I, P, color=c4)
        ax1.set_xlim(0, 75)
        ax1.set_ylim(0, 55)
        ax2.set_ylim(0, 3000)

        ax1.set_xlabel("Current [A]")
        ax1.set_ylabel("Voltage [V]", color=c2)
        ax2.set_ylabel("Power [W]", color=c4)

        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    fuelcell = FuelCell()
    fuelcell.plot()
