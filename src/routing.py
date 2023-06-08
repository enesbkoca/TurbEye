import numpy as np
import matplotlib.pyplot as plt
from src.shelf_drone import ShelfMotor, ShelfPropeller, ShelfESC
from src.drone import Drone
from src.speed_range import SpeedRange
from src.windfarm import WindFarm
from collections import Counter
import heapq
import json

class DroneRoute(SpeedRange):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.inspection_time = 0.5  # hrs
        self.mh2 = 0.12  # kg
        self.Cd = 0.8
        self.max_consumption = 0.9
        self.speed, self.angle = self.optimal_range_parameters(0.8)[1:]

        self.E_tot = self.mh2 * 34000 * self.max_consumption  # Wh
        self.E_ins = self.P_tot * self.inspection_time  # Wh
        P_cr = self.compute_endurance(1 / np.cos(self.angle * np.pi / 180), Ptot=True)  # W
        self.E_cr = P_cr / self.speed / 3600  # Wh/m

        self.max_dist = (self.E_tot - self.E_ins) / self.E_cr / 2

    def find_route(self, X, rand=False, n=2):
        # Calculate distances from origin
        distances = np.linalg.norm(X, axis=1)

        if np.max(distances) > self.max_dist:
            print(len(np.where(distances > self.max_dist)[0]), 'of the turbines is/are unreachable')
            X = np.delete(X, np.where(distances > self.max_dist), axis=0)
            distances = np.delete(distances, np.where(distances > self.max_dist))

        visited = []
        left = X.copy()
        distancesleft = distances.copy()
        route = []

        # Define the dynamic cluster size threshold
        while len(left) > 0:
            E = 0
            t = 0
            start = (0, 0)
            return_back = False
            closestindex = np.argmin(distancesleft)
            trip = [start]
            dest_dist = distancesleft[closestindex]
            distancesleft = np.delete(distancesleft, closestindex)
            while not return_back:
                dest = left[closestindex]
                left = np.delete(left, closestindex, axis=0)
                E += dest_dist * self.E_cr
                t += dest_dist / self.speed / 3600
                visited.append(dest)
                trip.append(dest.tolist())
                start = dest
                E += self.E_ins
                t += self.inspection_time
                if len(left) == 0:
                    E += np.linalg.norm(dest) * self.E_cr
                    t += np.linalg.norm(dest) / self.speed / 3600
                    break
                new_dist = np.linalg.norm(left - start, axis=1)
                if rand and len(new_dist) > n-1:
                    closest = heapq.nsmallest(n, new_dist)[np.random.randint(0, n)]
                    closestindex = np.where(new_dist == closest)[0][0]
                else:
                    closestindex = np.argmin(new_dist)
                pot_E = E + new_dist[closestindex] * self.E_cr + np.linalg.norm(left[closestindex]) * self.E_cr + self.E_ins
                if pot_E > self.E_tot:
                    E += np.linalg.norm(dest) * self.E_cr
                    return_back = True
                else:
                    dest_dist = new_dist[closestindex]
                    distancesleft = np.delete(distancesleft, closestindex)
            if E > self.E_tot:
                print('Range exceeds maximum', E, self.E_tot)
            trip.append((0, 0))
            route.append((trip, E / 34000, t))
        return route

    def plot_route(self, route):
        for trip, m, t in route:
            x = [i[0] for i in trip]
            y = [i[1] for i in trip]
            plt.scatter(x, y)
            plt.plot(x, y)
        plt.axis('equal')
        plt.xlabel('Distance')
        plt.show()

    def properties(self, route):
        props = dict()
        props['trips'] = len(route)
        ins_per_trip = [len(trip) - 2 for trip, m, t in route]
        props['counter'] = Counter(ins_per_trip)
        props['kg H2'] = sum(m for trip, m, t in route)
        props['Total flight time'] = sum(t for trip, m, t in route)
        props['hrs of flight time'] = [t for trip, m, t in route]

        return props

    def find_best_route(self, X, no_of_iterations=1000, no_of_neighbors=2, best=None):
        if best is None:
            best_route = self.find_route(X)
            best_properties = self.properties(best_route)
        else:
            best_route = best[0]
            best_properties = best[1]
        for i in range(no_of_iterations):
            random_route = self.find_route(X, rand=True, n=no_of_neighbors)
            random_properties = self.properties(random_route)
            if random_properties['trips'] < best_properties['trips'] or \
                    (random_properties['trips'] == best_properties['trips']
                     and random_properties['kg H2'] < best_properties['kg H2']):
                best_route = random_route
                best_properties = random_properties
        return best_route, best_properties

    def save_and_load(self, load=True, plot=True, no_of_iterations=1000, no_of_neighbors=2):
        if load:
            with open("../datasets/best_route.json", "r") as f:
                best = json.load(f)
        route, prop = self.find_best_route(X, no_of_iterations=no_of_iterations, no_of_neighbors=no_of_neighbors, best=best)
        if plot:
            self.plot_route(route)
        with open("../datasets/best_route.json", "w") as f:
            json.dump((route, prop), f)
        return route, prop

    def plot_time_hist(self, prop=None):
        if prop is None:
            with open("../datasets/best_route.json", "r") as f:
                timelist = json.load(f)[1]['hrs of flight time']
        else:
            timelist = prop['hrs of flight time']
        plt.hist(timelist)
        plt.show()

    def plot_trip_counter(self, prop=None):
        if prop is None:
            with open("../datasets/best_route.json", "r") as f:
                counter = json.load(f)[1]['counter']
        plt.bar(counter.keys(), counter.values())
        plt.show()

if __name__ == '__main__':
    hornsea = WindFarm()
    X = hornsea.coordinates
    X = X.astype('float')

    prop = ShelfPropeller("T-Motor NS 26x85")
    motor = ShelfMotor("T-Motor Antigravity MN6007II KV160")
    esc = ShelfESC("T-Motor FLAME 60A")

    d = DroneRoute(propeller=prop, motor=motor, esc=esc, tank_mass=1.65)
    d.save_and_load()



