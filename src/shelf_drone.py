import json
from src.propeller import Propeller
from src.motor import Motor
from src.drone import Drone
import numpy as np
import matplotlib.pyplot as plt
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
            data = pd.read_csv(f'../experimental_data/{self.name}.csv')
            data['Rotation speed (rpm)'] = data['Rotation speed (rpm)'].replace(0, np.nan).dropna()
            N = data['Rotation speed (rpm)']
            T = data['Thrust (kgf)'] * 9.80665
            M = data['Torque (N⋅m)']
            Ct = T/(self.rho * (N / 60) ** 2 * self.Dp**4)
            self.Ct = Ct.mean()
            Cm = M / (self.rho * (N / 60) ** 2 * self.Dp ** 5)
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


if __name__ == "__main__":
    prop = ShelfPropeller("T-Motor NS 26x85")
    motor = ShelfMotor("T-Motor Antigravity MN6007II KV160")

    drone = Drone(propeller=prop, motor=motor)
    print(drone.propeller)
    print(drone.motor)
    print(drone.mass)
    drone.validation()

