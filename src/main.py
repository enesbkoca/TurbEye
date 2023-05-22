from scipy.optimize import minimize
import numpy as np
from motor import Motor
from propeller import Propeller
from hydrogen import Hydrogen
from drone import Drone

if __name__ == "__main__":
    p = Propeller(0.5, 0.168, 2, 0.034)
    m = Motor(170, 10, 0.4, 0.27, 19, 0.128, p)
    d = Drone(3, 8, 2, p, m)
    print(d.compute_weight(x=[0.5, 0.168, 2, 170]))

    res = minimize(d.compute_weight, x0=np.array([0.5, 0.168, 2, 170]), bounds=[(0, 1), (0, 0.4), (2, 6), (1, 1000)],
                   method='Nelder-Mead')

    print(res)
