import json
from src.propeller import Propeller
from src.motor import Motor
from src.drone import Drone


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
    motor = ShelfMotor("T-Motor Antigravity MN6007II KV160")
    prop = ShelfPropeller("T-Motor MF2211")

    our_drone = Drone(motor=motor, propeller=prop)

    motor = ShelfMotor("T-Motor MN5212 KV340")
    prop = ShelfPropeller("T-Motor P18x61")

    book_drone = Drone(motor=motor, propeller=prop)

    book_drone.plot_PT()

    paper_prop = ShelfPropeller("T-Motor G40x13.1")

    # print(paper_prop.required_rpm(325))


    # motor = ShelfMotor("T-Motor MN5212 KV 340")


