from drone_combinator import DroneCombinator
from drone import Drone

if __name__ == "__main__":
    drone = Drone()
    # drone.plot_endurance_TW()
    drone.plot_PT()

    # combinations = DroneCombinator()
    # combinations.print_drones(count=20, upper_limit=15)
    # drone = combinations[0]
    #
    # drone.plot_PT()
