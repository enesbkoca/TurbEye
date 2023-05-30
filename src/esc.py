

class ESC:

    def __init__(self, Iemax, Re, mass):
       self.Iemax = Iemax
       self.Re = Re
       self.mass = mass

    def equivalentV(self, Vm, Im):
        return Vm + Im * self.Re

    def throttle(self):