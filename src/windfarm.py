import re
from functools import reduce
from decimal import Decimal
import numpy as np
import matplotlib.pyplot as plt
from src.shelf_drone import ShelfPropeller, ShelfMotor, ShelfESC
from src.speed_range import SpeedRange


class WindTurbine:
    def __init__(self, identifier, item, long, lat):
        self.id = identifier
        self.item = item
        self.long = long
        self.lat = lat

        self.x = self.transform_coordinate(self.long)
        self.y = self.transform_coordinate(self.lat)

        self.x_n = None
        self.y_n = None

    def transform_coordinate(self, coordinate):
        # transform to meters
        nums = re.findall(r'\d+\.\d+|\d+', coordinate)
        nums = list(map(lambda i: np.double(i), nums))

        degrees = (nums[2] / 60 + nums[1]) / 60 + nums[0]
        meters = 111139 * degrees
        return meters

    def get_xy(self, normalized=True):
        if normalized:
            return np.array([self.x_n, self.y_n])
        else:
            return np.array([self.x, self.y])

    def __str__(self):
        return f"{self.id} | {self.item}"

    __repr__ = __str__


class OSS(WindTurbine):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class WindFarm:

    def __init__(self):
        self.file = "../datasets/windfarm.csv"

        self.drone = SpeedRange(
            propeller=ShelfPropeller("T-Motor NS 26x85"),
            motor=ShelfMotor("T-Motor Antigravity MN6007II KV160"),
            esc=ShelfESC("T-Motor FLAME 60A"),
            tank_mass=1.65)
        self.turbines = []
        self.oss = None
        self.read_file()
        self.normalize()
        self.coordinates = self.windfarm_coordinates()

    def windfarm_coordinates(self):

        coordinates = np.empty((len(self.turbines), 2), dtype=np.ndarray)

        for idx, turbine in enumerate(self.turbines):
            coordinates[idx] = turbine.get_xy()

        return coordinates

    def read_file(self):
        with open(self.file, 'r') as f:
            header = f.readline()
            for line in f:
                identifier, item, long, lat = line.strip().split(",")

                if item == "OSS":
                    self.oss = OSS(identifier, item, long, lat)
                elif item == "WTG":
                    self.turbines.append(
                        WindTurbine(identifier, item, long, lat)
                    )

    def normalize(self):
        self.oss.y_n = 0
        self.oss.x_n = 0

        for turbine in self.turbines:
            turbine.y_n = turbine.y - self.oss.y
            turbine.x_n = turbine.x - self.oss.x

    def plot_farm(self, normalized=True):

        plt.plot(*self.oss.get_xy(normalized=True) / 1000, label="OSS", color="y", marker="D")

        for turbine in self.turbines:
            plt.plot(*turbine.get_xy(normalized=True) / 1000, color="m", marker=".")
            # plt.text(*turbine.get_xy(), turbine.id)

        # plt.legend()
        plt.axis('equal')
        plt.show()

    def get_turbine(self, identifier):
        for turbine in self.turbines:
            if turbine.id == identifier:
                return turbine
        else:
            print("Turbine not found")




if __name__ == "__main__":
    hornsea = WindFarm()
    hornsea.plot_farm()
    print(hornsea.coordinates)
