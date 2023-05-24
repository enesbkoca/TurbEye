import numpy as np
from matplotlib import pyplot as plt
from src.drone import Drone


class SensitivityAnalysis:
    def __init__(self, drone):
        self.initial_drone = drone
        self.initial_mass = drone.mass

        self.range = (
            np.array([-0.25, -0.2, -0.15, -0.1, -0.05, 0.05, 0.1, 0.15, 0.2, 0.25])
            * 100
        )

        self.parameters = list(self.initial_drone.config.keys())

        if "Bp" in self.parameters:
            self.parameters.remove("Bp")

        self.x_values = []
        self.mass_values = []

    def generate_drones(self, parameter):
        mass = []
        x = []

        for i in self.range:
            config = self.initial_drone.config
            config[parameter] *= 1 + i / 100

            drone = Drone(config)

            if drone.mass:
                mass.append(drone.mass)
                x.append(i)

        mass = np.array(mass)
        x = np.array(x)

        mass = (mass - self.initial_mass) / self.initial_mass * 100

        self.mass_values.append(mass)
        self.x_values.append(x)

    def perform_analysis(self):
        for parameter in self.parameters:
            self.generate_drones(parameter)

    def plot(self):
        fig = plt.figure()

        for parameter, x, y in zip(self.parameters, self.x_values, self.mass_values):
            Drone.plot(fig, x, y, "Parameter Diff [%]", "Mass Diff [%]", parameter)

        plt.legend(loc="upper right", ncols=3)
        plt.show()


if __name__ == "__main__":
    drone = Drone()

    sens = SensitivityAnalysis(drone)

    sens.perform_analysis()
    sens.plot()
