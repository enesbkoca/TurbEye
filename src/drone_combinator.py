import json
from prettytable import PrettyTable
from src.drone import Drone
from src.motor import Motor
from src.propeller import Propeller


class DroneCombinator:
    def __init__(self):
        self.drones = []
        self.motors = None
        self.propellers = None

        self.read_motor_prop()
        self.create_drones()
        self.sort_drones()

    def read_motor_prop(self):
        with open("../datasets/motor.json", "r") as file:
            self.motors = json.load(file)

        with open("../datasets/propeller.json", "r") as file:
            self.propellers = json.load(file)

    def create_drones(self):
        for motor_name, motor_properties in self.motors.items():
            for prop_name, prop_properties in self.propellers.items():
                motor = Motor(*motor_properties)
                motor.name = motor_name

                propeller = Propeller(*prop_properties)
                propeller.name = prop_name

                drone = Drone(propeller=propeller, motor=motor)

                if drone.mass:
                    self.drones.append(drone)

    def sort_drones(self):
        self.drones.sort(key=lambda drone: drone.mass)

        return self.drones

    def print_drones(self, count: int = 5, upper_limit: float = 12):
        table = PrettyTable(("i", "Propeller", "Motor", "Mass", "RPM", "Current Ratio"))

        for idx, drone in enumerate(self.drones[:count]):
            if drone.mass > upper_limit:
                break

            table.add_row(
                (
                    idx,
                    drone.propeller,
                    drone.motor,
                    f"{drone.mass:.2f} kg",
                    f"{drone.N:.2f} rpm",
                    f"{drone.I_ratio:.2f}",
                )
            )

        print(table)

    def __getitem__(self, item):
        return self.drones[item]


if __name__ == "__main__":
    combinations = DroneCombinator()
    combinations.print_drones(count=20, upper_limit=205)
