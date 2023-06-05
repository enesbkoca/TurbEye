import unittest
from src.shelf_drone import ShelfPropeller, ShelfMotor, ShelfESC
from src.drone import Drone
from src.propeller import Propeller
import pandas as pd
import numpy as np


class TestShelfDrone(unittest.TestCase):

    def setUp(self) -> None:
        self.P = ShelfPropeller('T-Motor NS 28x92')
        self.M = ShelfMotor('T-Motor Antigravity MN5008 KV340')
        self.ESC = ShelfESC('T-Motor FLAME 60A')
        self.D = Drone(propeller=self.P, motor=self.M, esc=self.ESC)

    def test_shelf_prop(self):
        self.assertTrue(self.P.Dp == 0.7112)
        self.assertTrue(self.P.Hp == 0.23368)
        self.assertTrue(self.P.name == 'T-Motor NS 28x92')

    def test_shelf_motor(self):
        self.assertTrue(self.M.Kv0 == 340)
        self.assertTrue(self.M.Um0 == 22)
        self.assertTrue(self.M.name == 'T-Motor Antigravity MN5008 KV340')

    def test_shelf_esc(self):
        self.assertTrue(self.ESC.Iecont == 60)
        self.assertTrue(self.ESC.Iemax == 80)

    def test_drone(self):
        self.assertTrue(self.D.propeller.Bp == 2)
        self.assertTrue(self.D.motor.Immax == 35)
        self.assertTrue(self.D.esc.mass == 0.0735)

    def test_cor_coeff(self):
        cor_prop = ShelfPropeller('T-Motor NS 22x66')
        prop = Propeller(0.5588, 0.1676, 2, 0.034)
        prop.name = 'T-Motor NS 22x66'
        self.assertNotEqual(prop.Cm, cor_prop.Cm)
        data = pd.read_csv(f'../experimental_data/{prop.name}.csv')
        data['Rotation speed (rpm)'] = data['Rotation speed (rpm)'].replace(0, np.nan).dropna()
        Ct = data['Thrust (kgf)'] * 9.80665 / (prop.rho * (data['Rotation speed (rpm)'] / 60) ** 2 * prop.Dp ** 4)
        self.assertAlmostEqual(Ct.mean(), cor_prop.Ct)


if __name__ == '__main__':
    unittest.main()
