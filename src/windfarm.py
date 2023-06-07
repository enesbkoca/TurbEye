import re
from functools import reduce
from decimal import Decimal
import matplotlib.pyplot as plt
from src.shelf_drone import ShelfPropeller, ShelfMotor, ShelfESC
from src.speed_range import SpeedRange


class WindTurbine:
    def __init__(self, identifier, item, long, lat):
        self.id = identifier
        self.item = item
        self.long = long
        self.lat = lat

        self.x = self.transform_coordinate(self.lat)
        self.y = self.transform_coordinate(self.long)

    def transform_coordinate(self, coordinate):
        nums = re.findall(r'\d+\.\d+|\d+', coordinate)
        nums = list(map(lambda i: Decimal(i), nums))
        # value = reduce(lambda a, b: (a + b/60), nums)

        value = (nums[2] / 60 + nums[1]) / 60 + nums[0]

        return value

    def get_xy(self):
        return self.x, self.y

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

    def plot_farm(self):
        plt.plot(*self.oss.get_xy(), label="OSS", color="g", marker="D")

        for turbine in self.turbines:
            plt.plot(*turbine.get_xy(), color="b", marker=".")
            plt.text(*turbine.get_xy(), turbine.id)

        # plt.legend()
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
    print()
