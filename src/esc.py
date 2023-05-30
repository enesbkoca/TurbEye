class ESC:
    def __init__(self, Iemax, Iecont, mass):
        self.Iemax = Iemax
        self.Iecont = Iecont
        self.mass = mass
        self.Re = self.resistance()
        self.name = "Custom ESC"

    def resistance(self):
        return 32.6754 * self.Iecont ** (-0.7669) + 0.5269

    def throttle(self, Um, Im, Ub):
        return (Um + Im * self.Re) / Ub

    # ESC current
    def inputI(self, Vm, Im, Ub):
        I = self.throttle(Vm, Im, Ub) * Im
        if I > self.Iemax:
            raise ValueError("ESC current too high: ", I)
        return I

    # ESC voltage
    def inputV(self, Ub, Ib, Rb):
        return Ub - Ib * Rb

    def __repr__(self):
        return self.name

    def __str__(self):
        return self.__repr__()

    @property
    def dict(self):
        return {"Iemax": self.Iemax,
                "Iecont": self.Iecont,
                "m_mass": self.mass}


if __name__ == "__main__":
    esc = ESC(120, 100, 0.07)
    print(esc.Re)
