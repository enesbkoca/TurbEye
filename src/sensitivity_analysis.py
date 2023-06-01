import numpy as np
from matplotlib import pyplot as plt
from src.drone import Drone
from copy import deepcopy


class SensitivityAnalysis:
    def __init__(self, drone):
        self.initial_drone = drone
        self.initial_mass = drone.mass

        self.range = np.arange(-0.25, 0.26, 0.05) * 100

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

    def perform_analysis(self):
        for type in self.types:
            for parameter in self.initial_drone.config[type]:
                self.generate_drones(type, parameter)

    def plot(self):
        fig = plt.figure()

        for parameter, x, y in zip(self.parameters, self.x_values, self.mass_values):
            Drone.plot(
                fig, x, y, "Parameter Diff [%]", "Mass Diff [%]", label=parameter
            )

        plt.legend(loc="upper right", ncols=3)
        plt.show()


if __name__ == "__main__":
    d = Drone()

    sens = SensitivityAnalysis(d)

    sens.perform_analysis()
    sens.plot()
