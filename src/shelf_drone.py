import json

from src.propeller import Propeller
from src.motor import Motor
from src.drone import Drone
from src.esc import ESC

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

    def cor_coeff(self, Cm, Ct):
        self.Cm = Cm
        self.Ct = Ct

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


class ShelfESC(ESC):
    ESCs = None

    def __init__(self, esc_name):
        self.read_esc()
        if esc_name not in self.ESCs:
            raise ValueError("ESC does not exist in database")

        super().__init__(*self.ESCs[esc_name])
        self.name = esc_name

    @classmethod
    def read_esc(cls):
        with open("../datasets/esc.json", "r") as file:
            cls.ESCs = json.load(file)


if __name__ == "__main__":
    prop = ShelfPropeller("T-Motor P18x61")
    motor = ShelfMotor("T-Motor Antigravity MN5008 KV170")
    esc = ShelfESC("T-Motor FLAME 60A")

    drone = Drone(propeller=prop, motor=motor)
    print(drone.propeller)
    print(drone.motor)
    drone.propeller.Cm = 0.005
    drone.propeller.Ct = 0.09
    drone.plot_PT()
