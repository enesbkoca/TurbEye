from math import pi


class Motor:
    def __init__(self, Kv0, Um0, Im0, Rm, Immax, mass):
        # Motor characteristics
        self.Kv0 = Kv0  # KV value of the motor
        self.Um0 = Um0  # No load Voltage
        self.Im0 = Im0  # No load current
        self.Rm = Rm  # Motor resistance
        self.Immax = Immax  # Motor max current
        self.mass = mass

        self.name = "Custom Motor"

    def VandI(self, M, N):
        Im = M * self.Kv0 * self.Um0 / 9.55 / (self.Um0 - self.Im0 * self.Rm) + self.Im0
        Um = Im * self.Rm + (self.Um0 - self.Im0 * self.Rm) / (self.Kv0 * self.Um0) * N

        return Um, Im

    def __repr__(self):
        return self.name

    def __str__(self):
        return self.__repr__()

    @property
    def dict(self):
        return {
            "Kv0": self.Kv0,
            "Um0": self.Um0,
            "Im0": self.Im0,
            "Rm": self.Rm,
            "Immax": self.Immax,
            "m_motor": self.mass,
        }


if __name__ == "__main__":
    ...
