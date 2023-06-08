import numpy as np
from src.windfarm import WindFarm


class VRPSolver:
    def __init__(self, windfarm):
        self.windfarm = windfarm
        self.coordinates = np.vstack((np.array([[0, 0]]), self.windfarm.coordinates)).astype('float')
        self.distance_matrix = self.compute_distance_matrix()

    def distance(self, from_index, to_index):
        return self.distance_matrix[from_index][to_index]

    def compute_distance_matrix(self):
        X = self.coordinates
        # x, y = self.coordinates[:, :1], self.coordinates[:, 1:]
        m = X.shape[0]
        dists = np.zeros((m, m))
        for i in range(m):
            dists[i, :] = np.sqrt(np.sum((X[i] - X) ** 2, axis=1))

        self.distance_matrix = dists

if __name__ == "__main__":
    hornsea = WindFarm()
    vrpsolver = VRPSolver(hornsea)
    print()