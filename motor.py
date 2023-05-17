from propeller import Propeller


class Motor:

    def __init__(self, Kv0, Um0, Im0, Rm, Immax, mass, Propeller):
        # Motor characteristics
        self.Kv0 = Kv0  # KV value of the motor
        self.Um0 = Um0  # No load Voltage
        self.Im0 = Im0  # No load current
        self.Rm = Rm  # Motor resistance
        self.Immax = Immax  # Motor max current
        self.mass = mass

        self.propeller = Propeller
        self.VandI()

    def VandI(self):
        M = self.propeller.M
        N = self.propeller.N
        self.Im = M*self.Kv0*self.Um0/9.55/(self.Um0-self.Im0*self.Rm)
        self.Um = self.Im*self.Rm + (self.Um0-self.Im0*self.Rm)/(self.Kv0*self.Um0)*N
