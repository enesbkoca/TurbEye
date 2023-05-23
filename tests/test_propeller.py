import unittest
from src.propeller import Propeller
import numpy as np


class PropTest(unittest.TestCase):
    def setUp(self) -> None:
        self.P = Propeller(0.5, 0.1, 2, 0.1)
        self.P.name = "Dummy"

    def test_init(self):
        self.assertTrue(self.P.Dp == 0.5)
        self.assertTrue(self.P.Hp == 0.1)
        self.assertTrue(self.P.Bp == 2)
        self.assertTrue(self.P.name == "Dummy")
        self.assertTrue(self.P.mass == 0.1)

    def test_coefficients(self):
        self.assertTrue(np.isclose(self.P.Ct == 0.04399))
        self.assertTrue(self.P.Cm == 0.003578)
        self.assertTrue(self.P.Cd == 0.019334)
