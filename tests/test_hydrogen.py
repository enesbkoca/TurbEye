import unittest
from src.hydrogen import Hydrogen


class HydTest(unittest.TestCase):
    def setUp(self) -> None:
        self.H = Hydrogen(34000)

    def test_init(self):
        self.assertTrue(self.H.U == 34000)
        self.assertTrue(self.H.mh2 == 1)

    def test_mass(self):
        self.assertTrue(self.H.tank_mass() == 41.187 + 8.38 + 1.0587)
        self.assertTrue(self.H.tot_mass() == 41.187 + 8.38 + 1.0587 + 1)
