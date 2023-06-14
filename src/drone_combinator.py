import json
from prettytable import PrettyTable
from src.drone import Drone
from src.shelf_drone import ShelfPropeller, ShelfMotor, ShelfESC

class DroneCombinator:
    motors = None
    propellers = None

    def __init__(self):
        self.drones = []

        self.read_motor_prop()
        self.create_drones()
        self.sort_drones()

    def read_motor_prop(self):
        with open("../datasets/motor.json", "r") as file:
            self.motors = json.load(file)

        with open("../datasets/propeller.json", "r") as file:
            self.propellers = json.load(file)

    def create_drones(self):
        for motor_name in self.motors:
            for prop_name in self.propellers:
                motor = ShelfMotor(motor_name)
                propeller = ShelfPropeller(prop_name)
                esc = ShelfESC("T-Motor FLAME 60A")
                drone = Drone(propeller=propeller, motor=motor, esc=esc)

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

        self.table = table
        print(table)

    def __getitem__(self, item):
        return self.drones[item]


if __name__ == "__main__":
    combinations = DroneCombinator()
    combinations.print_drones(count=20, upper_limit=15)
