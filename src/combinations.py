import json
from prettytable import PrettyTable
from drone import Drone
from motor import Motor
from propeller import Propeller


class Combinations:
    def __init__(self):
        self.drones = []
        self.motors = None
        self.propellers = None

        self.read_motor_prop()
        self.create_drones()

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

                if drone.mass and drone.mass < 12:
                    self.drones.append(drone)

    def sort_drones(self, count: int = 5):

        self.drones.sort(key=lambda drone: drone.mass)

        return self.drones[:count]

    def table_drones(self):

        table = PrettyTable(("Propeller", "Motor", "Mass", "RPM"))

        for drone in self.drones:
            table.add_row((drone.propeller, drone.motor, f"{drone.mass:.2f} kg", f"{drone.N:.2f} rpm"))

        return table
