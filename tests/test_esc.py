import unittest
from src.esc import ESC
from numpy import isclose

class ESCTest(unittest.TestCase):
    def setUp(self) -> None:
        self.esc = ESC(120, 100, 0.05)

    def test_init(self):
        esc = self.esc
        self.assertTrue(esc.Iemax == 120)
        self.assertTrue(esc.Iecont == 100)
        self.assertTrue(esc.mass == 0.05)
        self.assertTrue(esc.name == "Custom ESC")

    def test_resistance(self):
        esc_same_Imax = ESC(120, 40, 10)
        esc_diff_Imax = ESC(150, 100, 0.05)

        self.assertTrue(isclose(self.esc.Re, 0.0013581, rtol=1e-3, atol=1e-3))

        self.assertTrue(isclose(esc_same_Imax.Re, self.esc.Re))
        self.assertTrue(not isclose(esc_same_Imax.mass, self.esc.mass))

        self.assertTrue(not isclose(esc_diff_Imax.Re, self.esc.Re))
        self.assertTrue(isclose(esc_diff_Imax.mass, self.esc.mass))
        self.assertTrue(isclose(esc_diff_Imax.Iecont, self.esc.Iecont))

    def test_throttle(self):
        throttle_high = self.esc.throttle(50, 12, 70)
        self.assertTrue(isclose(throttle_high, 0.7145, rtol=1e-3, atol=1e-3))

        throttle_low = self.esc.throttle(20, 4, 70)
        self.assertTrue(isclose(throttle_low, 0.2858, rtol=1e-3, atol=1e-3), msg=throttle_low)

        self.assertTrue(throttle_high > throttle_low)
        self.assertTrue(throttle_high > 0 and throttle_low > 0)


if __name__ == "__main__":
    unittest.main()
