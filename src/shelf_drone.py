import json

from src.propeller import Propeller
from src.motor import Motor
from src.drone import Drone
from src.esc import ESC

import numpy as np
import pandas as pd


class ShelfPropeller(Propeller):
    propellers = None

    def __init__(self, prop_name):
        self.read_prop()

        if prop_name not in self.propellers:
            raise ValueError("Propeller does not exist in database")

        super().__init__(*self.propellers[prop_name])
        self.name = prop_name
        self.cor_coeff()

    def cor_coeff(self):
        try:
            data = pd.read_csv(f"../experimental_data/{self.name}.csv")
            data["Rotation speed (rpm)"] = (
                data["Rotation speed (rpm)"].replace(0, np.nan).dropna()
            )
            N = data["Rotation speed (rpm)"]
            T = data["Thrust (kgf)"] * 9.80665
            M = data["Torque (N⋅m)"]
            Ct = T / (self.rho * (N / 60) ** 2 * self.Dp**4)
            self.Ct = Ct.mean()
            Cm = M / (self.rho * (N / 60) ** 2 * self.Dp**5)
            self.Cm = Cm.mean()
        except FileNotFoundError:
            pass

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
    prop = ShelfPropeller("T-Motor NS 26x85")
    motor = ShelfMotor("T-Motor Antigravity MN6007II KV160")
    esc = ShelfESC("T-Motor FLAME 60A")

    drone = Drone(propeller=prop, motor=motor, esc=esc, tank_mass=1.65)
    print(drone.propeller)
    print(drone.motor)
    print(drone.mass)
    print(drone.compute_endurance(1, mh2=0.12))

    drone.plot_PT()
    drone.plot_ESC_FC()
