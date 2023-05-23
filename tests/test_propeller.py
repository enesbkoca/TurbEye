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
        self.assertTrue(self.P.__repr__() == 'Dummy')

    def test_coefficients(self):
        self.assertTrue(np.isclose(self.P.Ct, 0.04399, rtol=0.01, atol=0.001))
        self.assertTrue(np.isclose(self.P.Cm, 0.003578, rtol=0.01, atol=0.001))
        self.assertTrue(np.isclose(self.P.Cd, 0.019334, rtol=0.01, atol=0.001), msg=self.P.Cd)

    def test_forces(self):
        T, M = self.P.forces(600)
        self.assertTrue(np.isclose(T, 0.04399*1.225*100*0.5**4, rtol=0.01, atol=0.001), msg=T)
        self.assertTrue(np.isclose(M, 0.003578 * 1.225 * 100 * 0.5 ** 5, rtol=0.01, atol=0.001), msg=M)

    def test_req_rpm(self):
        N = self.P.required_rpm(1)
        self.assertTrue(np.isclose(N, (1/0.04399/1.225/(0.5**4))**0.5*60, rtol=0.01, atol=0.001), msg=N)


