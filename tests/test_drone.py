import unittest
from src.drone import Drone
import numpy as np


class TestDrone(unittest.TestCase):
    def setUp(self) -> None:
        config = {
    "mission": {"T": 1, "Nm": 4, "TW_R": 2},
    "propeller": {"Dp": 0.5, "Hp": 0.1, "Bp": 2, "m_prop": 0.1},
    "motor": {
        "Kv0": 100,
        "Um0": 20,
        "Im0": 0.7,
        "Rm": 0.2,
        "Immax": 24,
        "m_motor": 0.175,
    },
    "esc": {"Iemax": 80, "Iecont": 60, "mass": 0.006},
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

    def test_check_for_max(self):
        I = self.D.check_for_max(10, 1) * self.D.motor.Immax
        N = self.D.propeller.required_rpm(10 * 9.80665 * self.D.TW_R / self.D.Nm)
        M = self.D.propeller.forces(N)[1]
        Icalc = self.D.motor.VandI(M, N)[1]

        self.assertTrue(np.allclose(I, Icalc))

    def test_compute_endurance(self):
        av_t = np.arange(1, 2, 0.1)
        Time_model = self.D.compute_endurance(av_t, co_eff=1, tw_f=1, Ppay=0)
        E = self.D.hyd.mh2 * self.D.hyd.U
        T = self.D.mass * 9.80665 / self.D.Nm * av_t
        N = self.D.propeller.required_rpm(T)
        M = self.D.propeller.forces(N)[1]
        V, I = self.D.motor.VandI(M, N)
        P = V * I
        P_tot = P*self.D.Nm
        self.assertTrue(np.allclose(E/P_tot, Time_model))



if __name__ == "__main__":
    unittest.main()
