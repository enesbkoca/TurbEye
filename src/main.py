from drone import Drone
from combinations import Combinations

if __name__ == "__main__":
    drone = Drone()
    print(drone.mass)

    combinations = Combinations()

    print(combinations.table_drones(count=200, upper_limit=15))
