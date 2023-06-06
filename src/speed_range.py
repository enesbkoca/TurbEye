from numpy import arccos, pi, tan, cos, sin
import numpy as np
from src.drone import Drone
from shelf_drone import ShelfMotor, ShelfESC, ShelfPropeller

import matplotlib.pyplot as plt

class SpeedRange(Drone):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.S_x = 85000E-6  # Front surface area
        self.S_y = 14700E-6  # Side surface area
        self.S_z = 209500E-6  # Top surface area
        self.Cd = 2

        self.compute_sides()

    def max_pitch(self):
        return arccos(1/self.config['mission']['TW_R']) * 180 / pi

    def compute_sides(self):
        #      .+------+
        #    .' |    .'|     ab = S_x = front/back
        #   +---+--+'  |     bc = S_y = sides
        #   |   |  |   |     ac = S_z = top/bottom
        # b |  ,+--+---+
        #   |.'    | .' c
        #   +------+'
        #     a

        self.a = (self.S_x * self.S_z / self.S_y) ** 0.5
        self.b = (self.S_x * self.S_y / self.S_z) ** 0.5
        self.c = (self.S_y * self.S_z / self.S_x) ** 0.5
        self.r_corner = np.array([self.c/2, self.b/2])
        self.l_corner = np.array([-self.c/2, self.b/2])

    def rotation_matrix(self, angle):
        angle_rad = angle * pi / 180
        return np.array([
            [cos(angle_rad), -sin(angle_rad)],
            [sin(angle_rad), cos(angle_rad)]
        ])

    def surface_area(self, pitch):
        new_r_corner = np.abs(np.dot(self.r_corner, self.rotation_matrix(pitch).T))
        new_l_corner = np.abs(np.dot(self.l_corner, self.rotation_matrix(pitch).T))
        y = 2 * max(new_r_corner[1], new_l_corner[1])

        area = y * self.a
        return area

    def speed(self, pitch):
        pitch_rad = pitch * pi / 180
        return (2 * self.mass * 9.80665 * tan(pitch_rad) / (1.225 * self.surface_area(pitch) * self.Cd)) ** 0.5

    def range(self, pitch):
        pitch_rad = pitch * pi / 180
        TW_ratio = 1 / cos(pitch_rad)

        endurance = self.compute_endurance(TW_ratio)
        speed = self.speed(pitch)

        return endurance * speed * 3600

    def plot_graph(self, x, y, xlabel, ylabel):
        plt.plot(x, y)
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.tight_layout()
        plt.grid(which="minor", linewidth=0.2)
        plt.grid(which="major", linewidth=1)
        plt.minorticks_on()

        plt.show()

    def plot_area(self):
        x = np.arange(0, 180, 0.1)
        y = np.array([self.surface_area(angle) for angle in x])
        self.plot_graph(x, y, "Pitch angle [deg]", "Cross-section area [m2]")

    def plot_speed(self):
        x = np.arange(0, self.max_pitch(), 0.1)
        y = np.array([self.speed(angle) for angle in x])
        self.plot_graph(x, y, "Pitch angle [deg]", "Horizontal speed [m/s]")

    def plot_range(self):
        x = np.arange(0, self.max_pitch(), 0.1)
        y = np.array([self.range(angle) for angle in x]) / 1000
        self.plot_graph(x, y, "Pitch angle [deg]", "Range [km]")


if __name__ == "__main__":
    prop = ShelfPropeller("T-Motor NS 26x85")
    motor = ShelfMotor("T-Motor Antigravity MN6007II KV320")
    esc = ShelfESC("T-Motor FLAME 60A")

    sr = SpeedRange(propeller=prop, motor=motor, esc=esc, tank_mass=1.65)

    sr.plot_area()
    sr.plot_speed()
    sr.plot_range()
