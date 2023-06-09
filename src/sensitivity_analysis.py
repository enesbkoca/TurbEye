import numpy as np
from matplotlib import pyplot as plt
from src.drone import Drone
from copy import deepcopy
import matplotlib as mpl
from src.components import c1, c2, c3, c4, c5

mpl.rcParams['axes.prop_cycle'] = (mpl.cycler(color=[c3, c4, c5, c1, c2] * 2) +
                                   mpl.cycler(linestyle=['solid'] * 5 + ['dashed'] * 5))


class SensitivityAnalysis:
    def __init__(self, drone):
        self.initial_drone = drone
        self.initial_mass = drone.mass

        self.drones = []

        self.range = np.arange(-0.25, 0.26, 0.025) * 100

        self.types = list(self.initial_drone.config.keys())

        self.x_values = []
        self.mass_values = []
        self.parameters = []

    def generate_drones(self, type, parameter):
        if parameter in ["Nm", "Bp", "TW_R", "Immax"]:
            return

        mass = []
        x = []

        for i in self.range:
            config = deepcopy(self.initial_drone.config)
            config[type][parameter] *= 1 + i / 100

            drone = Drone(config)
            self.drones.append(drone)

            if drone.mass:
                mass.append(drone.mass)
                x.append(i)

        mass = np.array(mass)
        x = np.array(x)

        mass = (mass - self.initial_mass) / self.initial_mass * 100

        self.mass_values.append(mass)
        self.x_values.append(x)
        self.parameters.append(parameter)

    def perform_analysis(self, typeindex=None):
        if typeindex is not None:
            typ = self.types[typeindex]
            for parameter in self.initial_drone.config[typ]:
                self.generate_drones(typ, parameter)
        else:
            for typ in self.types:
                for parameter in self.initial_drone.config[typ]:
                    self.generate_drones(typ, parameter)

    def plot(self, refresh=True):
        fig = plt.figure()

        legend_names = {
            "Hp": "Propeller Pitch",
            "Dp": "Propeller Diameter",
            "m_prop": "Propeller Mass",
            "T": "Flight Time",
            "TW_R": "Thrust to Weight Ratio",
            "Kv0": "KV Value",
            "Um0": "No load Voltage",
            "Im0": "No load Current",
            "Rm": "Motor Resistance",
            "Immax": "Maximum Current",
            "m_motor": "Motor Mass",
        }

        for parameter, x, y in zip(self.parameters, self.x_values, self.mass_values):
            legend_name = legend_names[parameter]
            Drone.plot(
                fig, x, y, "Parameter Diff [%]", "Mass Diff [%]", label=legend_name
            )
        plt.legend(ncols=2, loc='lower right')
        plt.tight_layout()
        plt.ylim(-5, None)
        plt.show()

        if refresh:
            self.x_values = []
            self.mass_values = []
            self.parameters = []


if __name__ == "__main__":
    d = Drone()

    sens = SensitivityAnalysis(d)

    for i in range(3):
        sens.perform_analysis(typeindex=i)
    sens.plot()
