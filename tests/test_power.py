import unittest
from numpy import isclose
from src.power import HydrogenTank, FuelCell


class TankTest(unittest.TestCase):
    def setUp(self) -> None:
        self.H = HydrogenTank(34000)

    def test_init(self):
        self.assertTrue(self.H.U == 34000)
        self.assertTrue(self.H.mh2 == 1)

    def test_mass(self):
        self.assertTrue(self.H.tank_mass() == 41.187 + 8.38 + 1.0587)
        self.assertTrue(self.H.tot_mass() == 41.187 + 8.38 + 1.0587 + 1)


class FuelCellTest(unittest.TestCase):
    def setUp(self) -> None:
        self.fc = FuelCell()

    def test_init_(self):
        self.assertTrue(0 < self.fc.Imax < 150)

    def test_voltage(self):
        for I in range(0, self.fc.Imax):
            voltage = self.fc.get_voltage(I)
            self.assertTrue(voltage > 0)

    def test_voltage_error(self):
        with self.assertRaises(Exception) as context:
            high_I = self.fc.Imax + 1
            self.fc.get_voltage(high_I)

        self.assertTrue(f'Current larger than maximum: {high_I}' in str(context.exception))

        with self.assertRaises(Exception) as context:
            negative_I = -1
            self.fc.get_voltage(negative_I)

        self.assertTrue(f"Cannot get voltage from current: {negative_I}" in str(context.exception))

    def test_current(self):
        low_power = 50
        current, voltage = self.fc.get_current_voltage(low_power)

        self.assertTrue(isclose(current * voltage, low_power, rtol=1e-3, atol=1e-3), msg=(current * voltage, low_power))
        self.assertTrue(current <= self.fc.Imax)

        high_power = 8000
        current, voltage = self.fc.get_current_voltage(high_power)

        self.assertTrue(not isclose(current * voltage, high_power, rtol=1e-3, atol=1e-3), msg=(current * voltage, high_power))
        self.assertTrue(current <= self.fc.Imax)


if __name__ == "__main__":
    unittest.main()
