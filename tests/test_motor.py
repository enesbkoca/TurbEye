import unittest
import numpy as np
from src.motor import Motor


class MotorTest(unittest.TestCase):
    def setUp(self):
        self.motor = Motor(100, 20, 0.7, 0.2, 24, 0.175)
        self.motor.name = "Some Random Motor"

    def test_init(self):
        self.assertTrue(self.motor.Kv0 == 100)
        self.assertTrue(self.motor.Um0 == 20)
        self.assertTrue(self.motor.Im0 == 0.7)
        self.assertTrue(self.motor.Rm == 0.2)
        self.assertTrue(self.motor.Immax == 24)
        self.assertTrue(self.motor.mass == 0.175)
        self.assertTrue(self.motor.name == "Some Random Motor")

    def test_repr(self):
        self.assertTrue(self.motor.name == self.motor.__repr__())

    def test_VandI(self):
        self.assertTrue(
            np.allclose(
                self.motor.VandI(0.2378, 2500), (3.2075, 25.467), rtol=0.01, atol=0.001
            ),
            msg=self.motor.VandI(0.2378, 2500),
        )
