import json
from src.propeller import Propeller
from src.motor import Motor
from src.drone import Drone
import numpy as np
import matplotlib.pyplot as plt


class ShelfPropeller(Propeller):
    propellers = None

    def __init__(self, prop_name):
        self.read_prop()

        if prop_name not in self.propellers:
            raise ValueError("Propeller does not exist in database")

        super().__init__(*self.propellers[prop_name])
        self.name = prop_name

    @classmethod
    def read_prop(cls):
        with open("../datasets/propeller.json", "r") as file:
            cls.propellers = json.load(file)


class ShelfMotor(Motor):
    motors = None

    def __init__(self, motor_name):
        self.read_motor()
        if motor_name not in self.motors:
            raise ValueError("Motor does not exist in database")

        super().__init__(*self.motors[motor_name])
        self.name = motor_name

    @classmethod
    def read_motor(cls):
        with open("../datasets/motor.json", "r") as file:
            cls.motors = json.load(file)


if __name__ == "__main__":
    prop = ShelfPropeller("T-Motor NS 26x85")
    motor = ShelfMotor("T-Motor Antigravity MN6007II KV160")

    drone = Drone(propeller=prop, motor=motor)
    print(drone.propeller)
    print(drone.motor)
    drone.propeller.Cm = 0.00345
    drone.propeller.Ct = 0.0735
    drone.plot_PT()

