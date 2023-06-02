import numpy as np
import matplotlib.pyplot as plt


class HydrogenTank:
    def __init__(self, E):
        self.U = 34000
        self.mh2 = E / self.U

    def tank_mass(self):
        return 41.187 * self.mh2**2 + 8.38 * self.mh2 + 1.0587

    def tot_mass(self):
        return self.mh2 + self.tank_mass()


class StepUpConverter:
    def __init__(self):
        self.n_efficiency = 0.95
        self.V_output = 55
        self.I_output_max = 110

    def step_up(self, I_fc, V_fc):
        power = V_fc * I_fc * self.n_efficiency

        I_output = min(power / self.V_output, self.I_output_max)

        return I_output, self.V_output


class FuelCell:
    def __init__(self):
        self.Imax = 75
        self.converter = StepUpConverter()

    def getV(self, I):
        def left_linear(x):
            return 53 - 1.4151 * x

        def right_linear(x):
            return 46.4505 - 0.17934 * x

        if I > self.Imax:
            raise ValueError("Current larger than maximum: ", I)
        elif I <= 5.3:
            return left_linear(I)
        else:
            return right_linear(I)

    def getIandV(self, power):
        I = 0
        step = 0.1
        prev_power = 0

        while I < self.Imax:
            V = self.getV(I)
            curr_power = I * V

            if abs(curr_power - power) > abs(prev_power - power):
                I_output, V_output = self.converter.step_up(I, V)
                return I_output, V_output

            prev_power = curr_power
            I += step

        I_output, V_output = self.converter.step_up(I, V)

        return I_output, V_output

    def plot(self):
        I = np.arange(0, self.Imax, 0.1)
        get_voltage = np.vectorize(lambda x: self.getV(x))
        V = get_voltage(I)
        P = I * V

        converter_vectorized = np.vectorize(lambda x: self.getIandV(x))
        I_converter, V_converter = converter_vectorized(P)

        P_converter = I_converter * V_converter

        fig, ax1 = plt.subplots()

        plt.grid(which="minor", axis="both", linewidth=0.2)
        plt.grid(which="major", axis="both", linewidth=1)
        plt.minorticks_on()

        ax2 = ax1.twinx()
        ax1.plot(I, V, 'y')
        ax1.plot(I_converter, V_converter, 'y-.')
        ax2.plot(-2, -2, 'k-.', label='Converter')
        ax2.plot(-1, -1, 'k', label='Fuel Cell')

        ax2.plot(I, P, 'm')
        ax2.plot(I_converter, P_converter, 'm-.')
        ax1.set_xlim(0, 75)
        ax1.set_ylim(0, 60)
        ax2.set_ylim(0, 3000)

        ax1.set_xlabel('Current [A]')
        ax1.set_ylabel('Voltage [V]', color='y')
        ax2.set_ylabel('Power [W]', color='m')

        plt.legend()
        plt.tight_layout()
        plt.show()





if __name__ == "__main__":
    fuelcell = FuelCell()
    fuelcell.plot()
