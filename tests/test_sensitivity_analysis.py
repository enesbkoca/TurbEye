import unittest

import numpy as np
from src.drone import Drone
from src.sensitivity_analysis import SensitivityAnalysis
from src.shelf_drone import ShelfPropeller, ShelfMotor, ShelfESC


class SensitivityTest(unittest.TestCase):
    def setUp(self) -> None:
        propeller = ShelfPropeller("T-Motor NS 24x72")
        motor = ShelfMotor("T-Motor Antigravity MN6007II KV320")
        esc = ShelfESC("T-Motor FLAME 80A")
        self.drone = Drone(propeller=propeller, motor=motor, esc=esc)
        self.sa = SensitivityAnalysis(self.drone)

    def test_init(self):
        sa = self.sa

        self.assertTrue(isinstance(sa.initial_drone, Drone))
        self.assertEqual(sa.initial_drone, self.drone)

        self.assertTrue(sa.initial_mass > 0)

    def test_generate_drones(self):
        self.assertTrue(
            self.sa.x_values == self.sa.mass_values == self.sa.parameters == []
        )

        self.sa.generate_drones("propeller", "Hp")

        self.assertTrue(self.sa.parameters == ["Hp"])

        self.sa.perform_analysis()

        self.assertTrue(
            len(self.sa.x_values)
            == len(self.sa.mass_values)
            == len(self.sa.parameters)
            > 0
        )

        self.assertTrue(
            "Bp" not in self.sa.parameters and "Nm" not in self.sa.parameters
        )


if __name__ == "__main__":
    unittest.main()
