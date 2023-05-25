import json
from src.propeller import Propeller
from src.motor import Motor


class ShelfPropeller(Propeller):
    propellers = None

    def __new__(cls, prop_name):
        cls.read_prop()

        if prop_name not in cls.propellers:
            raise ValueError("Propeller does not exist in database")

        return super().__new__(cls)

    def __init__(self, prop_name):
        super().__init__(*self.propellers[prop_name])
        self.name = prop_name

    @classmethod
    def read_prop(cls):
        with open("../datasets/propeller.json", "r") as file:
            cls.propellers = json.load(file)


class ShelfMotor(Motor):
    motors = None

    def __new__(cls, prop_name):
        cls.read_motor()

        if prop_name not in cls.motors:
            raise ValueError("Motor does not exist in database")

        return super().__new__(cls)

    def __init__(self, motor_name):
        super().__init__(*self.motors[motor_name])
        self.name = motor_name

    @classmethod
    def read_motor(cls):
        with open("../datasets/motor.json", "r") as file:
            cls.motors = json.load(file)


if __name__ == "__main__":
    prop = ShelfPropeller("T-Motor MF2211")
    motor = ShelfMotor("T-Motor U15L KV43")