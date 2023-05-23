from drone import Drone
from dronecombinator import DroneCombinator

if __name__ == "__main__":
    drone = Drone()
    print(drone.mass)

    combinations = DroneCombinator()

    print(combinations.table_drones(count=200, upper_limit=15))
