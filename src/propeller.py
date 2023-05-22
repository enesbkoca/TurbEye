import numpy as np


# Aerodynamic characteristics
rho = 1.225  # Air density
A = 5  # Aspect Ratio
epsilon = 0.85  # Oswald efficiency factor
labda = 0.75
zeta = 0.5
e = 0.83
Cfd = 0.015  # Zero lift drag Coefficient
alpha0 = 0
K0 = 6.11


class Propeller:

    def __init__(self, Dp, Hp, Bp, mass):
        # Propeller characteristics
        self.Dp = Dp  # Propeller diameter
        self.Hp = Hp  # Propeller pitch
        self.Bp = Bp  # Number of blades in propeller

        self.mass = mass

        self.Ct, self.Cd, self.Cm = self.coefficients()

    def coefficients(self):
        Ct = (
            0.25
            * np.pi**3
            * labda
            * zeta**2
            * self.Bp
            * K0
            * (epsilon * np.arctan2(self.Hp, np.pi * self.Dp) - alpha0)
            / (np.pi * A + K0)
        )
        Cd = (
            Cfd
            + np.pi
            * A
            * K0**2
            / e
            * (epsilon * np.arctan2(self.Hp, np.pi * self.Dp) - alpha0) ** 2
            / (np.pi * A + K0) ** 2
        )
        Cm = 1 / 8 / A * np.pi**2 * Cd * zeta**2 * labda * self.Bp**2
        return Ct, Cd, Cm

    def forces(self, N):
        T = self.Ct * rho * (N / 60) ** 2 * self.Dp**4
        M = self.Cm * rho * (N / 60) ** 2 * self.Dp**5
        return T, M

    def required_rpm(self, T):
        return 60 * np.sqrt(T / self.Ct / rho / self.Dp**4)


