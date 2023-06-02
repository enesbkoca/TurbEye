import unittest
from src.power import HydrogenTank, StepUpConverter


class TankTest(unittest.TestCase):
    def setUp(self) -> None:
        self.H = HydrogenTank(34000)

    def test_init(self):
        self.assertTrue(self.H.U == 34000)
        self.assertTrue(self.H.mh2 == 1)

    def test_mass(self):
        self.assertTrue(self.H.tank_mass() == 41.187 + 8.38 + 1.0587)
        self.assertTrue(self.H.tot_mass() == 41.187 + 8.38 + 1.0587 + 1)


class ConverterTest(unittest.TestCase):
    def setUp(self):
        self.converter = StepUpConverter()

    def test_init(self):
        self.assertTrue(0 < self.converter.n_efficiency < 1)
        self.assertTrue(0 < self.converter.V_output < 100)
        self.assertTrue(0 < self.converter.I_output_max < 200)

    def test_step_up(self):
        I_fuelcell = 20
        V_fuelcell = 40
        P_fuelcell = I_fuelcell *  V_fuelcell

        I_output, V_output = self.converter.step_up(I_fuelcell, V_fuelcell)

        self.assertTrue(V_output == self.converter.V_output)
        self.assertTrue(I_output == P_fuelcell * self.converter.n_efficiency / V_output)

    def test_step_up_limit(self):
        I_fuelcell = 80
        V_fuelcell = 120

        I_output, V_output = self.converter.step_up(I_fuelcell, V_fuelcell)

        self.assertTrue(I_output == self.converter.I_output_max)
        self.assertTrue(V_output == self.converter.V_output)


if __name__ == "__main__":
    unittest.main()
