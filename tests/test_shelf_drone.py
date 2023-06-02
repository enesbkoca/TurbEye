import unittest
from src.shelf_drone import ShelfPropeller, ShelfMotor, ShelfESC
from src.drone import Drone


class TestShelfDrone(unittest.TestCase):

    def setUp(self) -> None:
        self.P = ShelfPropeller('T-Motor NS 28x92')
        M = ShelfMotor('T-Motor Antigravity MN5008 KV340')
        ESC = ShelfESC('T-Motor FLAME 60A')
        D = Drone(propeller=P, motor=M, esc=ESC)

    def test_shelf_prop(self):
        self.D.


if __name__ == '__main__':
    unittest.main()