import numpy as np
import matplotlib.pyplot as plt
from src.shelf_drone import ShelfMotor, ShelfPropeller, ShelfESC
from src.drone import Drone
from src.speed_range import SpeedRange
import heapq

inspection_time = 0.5  #hrs
mh2 = 0.12  #kg

prop = ShelfPropeller("T-Motor NS 26x85")
motor = ShelfMotor("T-Motor Antigravity MN6007II KV160")
esc = ShelfESC("T-Motor FLAME 60A")
drone = Drone(propeller=prop, motor=motor, esc=esc, tank_mass=1.65)

s = SpeedRange(propeller=prop, motor=motor, esc=esc, tank_mass=1.65)
speed, angle = s.optimal_range_parameters(0.8)[1:]  # Max range by safety factor

E_tot = mh2*34000  # Wh
E_ins = drone.P_tot * inspection_time  # Wh
P_cr = drone.compute_endurance(1/np.cos(angle*np.pi/180), Ptot=True)  # W
E_cr = P_cr / speed / 3600  # Wh/m

max_dist = (E_tot-E_ins)/E_cr/2

# Generate sample data
X = np.random.randn(147, 2) * 30000  # Replace with your own dataset

# Calculate distances from origin
distances = np.linalg.norm(X, axis=1)

if np.max(distances) > max_dist:
    print(len(np.where(distances > max_dist)[0]), 'of the turbines is/are unreachable')
    X = np.delete(X, np.where(distances > max_dist), axis=0)
    distances = np.delete(distances, np.where(distances > max_dist))

visited = []
left = X.copy()
distancesleft = distances.copy()
route = []
t = 0

# Define the dynamic cluster size threshold
while len(left) > 0:
    E = 0
    start = (0, 0)
    return_back = False
    closestindex = np.argmin(distancesleft)
    trip = [start]
    dest_dist = distancesleft[closestindex]
    distancesleft = np.delete(distancesleft, closestindex)
    while not return_back:
        dest = left[closestindex]
        left = np.delete(left, closestindex, axis=0)
        E += dest_dist * E_cr
        t += dest_dist / speed / 3600
        visited.append(dest)
        trip.append(dest)
        start = dest
        E += E_ins
        t += inspection_time
        if len(left) == 0:
            E += np.linalg.norm(dest) * E_cr
            t += np.linalg.norm(dest) / speed / 3600
            break
        new_dist = np.linalg.norm(left - start, axis=1)
        closestindex = np.argmin(new_dist)
        pot_E = E + new_dist[closestindex]*E_cr + np.linalg.norm(left[closestindex])*E_cr + E_ins
        if pot_E > E_tot:
            E += np.linalg.norm(dest) * E_cr
            return_back = True
        else:
            dest_dist = new_dist[closestindex]
            distancesleft = np.delete(distancesleft, closestindex)
    if E > E_tot:
        print('Range exceeds maximum', E, E_tot)
    trip.append((0, 0))
    x = [i[0] for i in trip]
    y = [i[1] for i in trip]
    plt.scatter(x, y)
    plt.plot(x, y)
    route.append((trip, E/34000))

print(len(route), 'trips')
print(sum(n for _, n in route), 'kg H2')
print(t, 'hrs of flight time')
plt.show()
