from drone import Drone
from combinations import Combinations

if __name__ == "__main__":
    drone = Drone()

    print(drone.mass)

    combinations = Combinations()

    drone_list = combinations.sort_drones(1000)

    print(combinations.table_drones())
