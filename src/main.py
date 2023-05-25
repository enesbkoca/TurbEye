from drone_combinator import DroneCombinator
from sensitivity_analysis import SensitivityAnalysis
from drone import Drone

if __name__ == "__main__":
    # drone = Drone()
    # drone.plot_endurance_TW()
    # drone.plot_PT()

    combinations = DroneCombinator()
    combinations.print_drones()
    # for d in combinations.drones[:1]:
    #     sens = SensitivityAnalysis(d)
    #     sens.perform_analysis()
    #     sens.plot()
