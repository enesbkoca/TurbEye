import unittest
from math import inf
from src.drone_combinator import DroneCombinator


class TestCombinator(unittest.TestCase):
    def setUp(self) -> None:
        self.combinator = DroneCombinator()

    def test_init(self):
        self.assertTrue(self.combinator.drones is not None)
        self.assertTrue(self.combinator.motors is not None)
        self.assertTrue(self.combinator.propellers is not None)

    def test_create_drones(self):
        n_combinations = len(self.combinator.motors) * len(self.combinator.propellers)

        self.assertTrue(len(self.combinator.drones) <= n_combinations)

        for drone in self.combinator.drones:
            self.assertTrue(drone.mass > 0)
            self.assertTrue(drone.propeller.name is not None)
            self.assertTrue(drone.motor.name is not None)

    def test_sorted_drones(self):
        current_weight = 0

        for drone in self.combinator.drones:
            self.assertTrue(drone.mass > current_weight)
            current_weight = drone.mass
