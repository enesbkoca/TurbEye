from numpy import arccos, pi, tan, cos, sin
from src.drone import Drone
from shelf_drone import ShelfMotor, ShelfESC, ShelfPropeller


class SpeedRange(Drone):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.Cd1 = 1
        self.Cd2 = 1
        self.S_max = 209500E-6

    def max_pitch(self):
        return arccos(1/self.config['mission']['TW_R']) * 180 / pi

    def Cd(self, pitch):
        pitch_rad = pitch * pi / 180
        return self.Cd1 * (1 - sin(pitch_rad) ** 3) + self.Cd2 * (1 - cos(pitch_rad) ** 3)

    def speed(self, pitch):
        pitch_rad = pitch * pi / 180
        return (2 * self.mass * 9.80665 * tan(pitch_rad) / (1.225 * self.S_max * self.Cd(pitch))) ** 0.5


if __name__ == "__main__":
    prop = ShelfPropeller("T-Motor NS 26x85")
    motor = ShelfMotor("T-Motor Antigravity MN6007II KV320")
    esc = ShelfESC("T-Motor FLAME 60A")

    sr = SpeedRange(propeller=prop, motor=motor, esc=esc, tank_mass=1.65)

    print(sr.speed(40))
