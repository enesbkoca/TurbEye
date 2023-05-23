class Hydrogen:
    def __init__(self, E):
        self.U = 34000
        self.mh2 = E / self.U

    def tank_mass(self):
        return 41.187 * self.mh2**2 + 8.38 * self.mh2 + 1.0587

    def tot_mass(self):
        return self.mh2 + self.tank_mass()
