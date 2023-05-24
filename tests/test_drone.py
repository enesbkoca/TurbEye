import unittest
from src.drone import Drone
import numpy as np


class TestDrone(unittest.TestCase):
    def setUp(self) -> None:
        config = {
            "T": 1,
            "Nm": 4,
            "TW_R": 2,
            "Dp": 0.5,
            "Hp": 0.1,
            "Bp": 2,
            "m_prop": 0.1,
            "Kv0": 100,
            "Um0": 20,
            "Im0": 0.7,
            "Rm": 0.2,
            "Immax": 24,
            "m_motor": 0.175,
        }
        self.D = Drone(config=config)

    def test_init(self):
        self.assertTrue(self.D.propeller.Dp == 0.5)
        self.assertTrue(self.D.propeller.Bp == 2)
        self.assertTrue(self.D.motor.Kv0 == 100)
        self.assertTrue(self.D.motor.Rm == 0.2)
        self.assertTrue(self.D.T == 1)
        self.assertTrue(self.D.Nm == 4)

    def test_mass(self):
        self.assertTrue(
            np.isclose(self.D.mass, 9.625, atol=0.01, rtol=0.1), msg=self.D.mass
        )


if __name__ == "__main__":
    unittest.main()
