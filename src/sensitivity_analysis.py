import numpy as np
from matplotlib import pyplot as plt
from src.drone import Drone
from copy import deepcopy


class SensitivityAnalysis:
    def __init__(self, drone):
        self.initial_drone = drone
        self.initial_mass = drone.mass

        self.range = np.arange(-0.25, 0.26, 0.025) * 100

        self.types = list(self.initial_drone.config.keys())

        self.x_values = []
        self.mass_values = []
        self.parameters = []

    def generate_drones(self, type, parameter):
        if "Bp" == parameter or "Nm" == parameter:
            return

        mass = []
        x = []

        for i in self.range:
            config = deepcopy(self.initial_drone.config)
            config[type][parameter] *= 1 + i / 100

            drone = Drone(config)

            if drone.mass:
                mass.append(drone.mass)
                x.append(i)

        mass = np.array(mass)
        x = np.array(x)

        mass = (mass - self.initial_mass) / self.initial_mass * 100

        self.mass_values.append(mass)
        self.x_values.append(x)
        self.parameters.append(parameter)

    def perform_analysis(self, typindex=None):
        if typindex is not None:
            typ = self.types[typindex]
            for parameter in self.initial_drone.config[typ]:
                self.generate_drones(typ, parameter)
        else:
            for typ in self.types:
                for parameter in self.initial_drone.config[typ]:
                    self.generate_drones(typ, parameter)

    def plot(self, refresh=True):
        fig = plt.figure()

        legend_names = {'Hp': 'Propeller Pitch', 'Dp': 'Propeller Diameter', 'm_prop': 'Propeller Mass', 'T': 'Flight Time', 'TW_R': 'Thrust to Weight Ratio',
                        'Kv0': 'KV Value', 'Um0': 'No load Voltage', 'Im0': 'No load Current', 'Rm': 'Motor Resistance',
                        'Immax': 'Maximum Current', 'm_motor': 'Motor Mass'}

        for parameter, x, y in zip(self.parameters, self.x_values, self.mass_values):
            legend_name = legend_names[parameter]
            Drone.plot(
                fig, x, y, "Parameter Diff [%]", "Mass Diff [%]", label=legend_name
            )
        plt.legend()
        plt.tight_layout()
        plt.show()

        if refresh:
            self.x_values = []
            self.mass_values = []
            self.parameters = []


if __name__ == "__main__":
    d = Drone()

    sens = SensitivityAnalysis(d)

    for i in range(3):
        sens.perform_analysis(typindex=i)
        sens.plot()
