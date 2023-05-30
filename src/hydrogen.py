class Hydrogen:
    def __init__(self, E):
        self.U = 34000
        self.mh2 = E / self.U

    def tank_mass(self):
        return 41.187 * self.mh2**2 + 8.38 * self.mh2 + 1.0587

    def tot_mass(self):
        return self.mh2 + self.tank_mass()


class FuelCell:
    def __init__(self):
        self.Imax = 75

    def getV(self, I):
        def left_linear(x):
            return 53 - 1.4151 * x

        def right_linear(x):
            return 46.4505 - 0.17934 * x

        if I > 75:
            raise ValueError("Current larger than maximum: ", I)
        elif I <= 5.3:
            return left_linear(I)
        else:
            return right_linear(I)

    def getIandV(self, power):
        I = 0
        step = 0.1
        prev_power = 0

        while I < 75:

            V = self.getV(I)
            curr_power = I * V

            if abs(curr_power - power) > abs(prev_power-power):
                return I, V

            prev_power = curr_power
            I += step

        return I, V
