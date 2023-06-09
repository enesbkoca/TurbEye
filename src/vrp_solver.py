import json
import numpy as np
from src.windfarm import WindFarm

from src.shelf_drone import ShelfMotor, ShelfESC, ShelfPropeller
from src.routing import DroneRoute

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


class VRPSolver:
    def __init__(self, windfarm, drone_route, n_trips=35):
        self.windfarm = windfarm
        self.drone = drone_route
        self.n_trips = n_trips
        self.n_turbines = len(self.windfarm.turbines)
        self.coordinates = np.vstack((np.array([[0, 0]]), self.windfarm.coordinates)).astype('float')
        self.distance_matrix = self.compute_distance_matrix()

    def distance(self, from_index, to_index):
        return self.distance_matrix[from_index][to_index]

    def compute_distance_matrix(self):
        X = self.coordinates
        m = X.shape[0]
        dists = np.zeros((m, m))
        for i in range(m):
            dists[i, :] = np.sqrt(np.sum((X[i] - X) ** 2, axis=1))

        return dists

    def create_data_model(self):
        data = dict()
        data["distance_matrix"] = self.distance_matrix
        data["num_vehicles"] = self.n_trips
        data["depot"] = 0
        data["distance_spent_turbine"] = int(self.drone.E_ins / self.drone.E_cr)  # Equivalent distance spent in turbine
        data["vehicle_cruise_speed"] = int(self.drone.speed)  # m/s
        data["vehicle_max_distance"] = int(data["vehicle_cruise_speed"] * 60 * 60 * 3 * self.drone.max_consumption) # Assuming 90% range is used (60 -> 54 seconds)
        data["n_locations"] = len(data["distance_matrix"])
        return data

    def print_solution(self, data, manager, routing, solution):
        """Prints solution on console."""
        # print(f'Objective: {solution.ObjectiveValue()}')
        total_distance = 0
        max_route_distance = 0
        routes = []
        total_hydro = 0
        total_hours = 0
        for vehicle_id in range(data['num_vehicles']):
            trip = []

            index = routing.Start(vehicle_id)
            plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
            route_distance = 0
            while not routing.IsEnd(index):
                trip.append(list(self.coordinates[manager.IndexToNode(index)]))
                plan_output += ' {} -> '.format(manager.IndexToNode(index))
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                route_distance += routing.GetArcCostForVehicle(
                    previous_index, index, vehicle_id)
            trip.append(list(self.coordinates[manager.IndexToNode(index)]))
            actual_distance = route_distance - data['distance_spent_turbine'] * (len(trip) - 2)
            hydro = (actual_distance * self.drone.E_cr + (len(trip) - 2) * self.drone.E_ins) / 34000
            total_hydro += hydro

            time = (actual_distance / data['vehicle_cruise_speed'] + (len(trip) - 2) * self.drone.inspection_time * 60 * 60) / 3600
            total_hours += time
            routes.append([trip, hydro, time])
            plan_output += '{}\n'.format(manager.IndexToNode(index))
            plan_output += 'Distance of the route: {}m\n'.format(route_distance)
            print(plan_output)
            total_distance += route_distance
            max_route_distance = max(route_distance, max_route_distance)

        with open("../datasets/vrp_solution.json", "w") as f:
            json.dump((routes, []), f)

        total_distance -= len(self.windfarm.turbines) * data['distance_spent_turbine']
        print('Maximum of the route distances: {}m'.format(max_route_distance))
        print(f'Total distance flown: {total_distance / 1000} km')
        total_time = len(self.windfarm.turbines) * self.drone.inspection_time * 60 + total_distance / data['vehicle_cruise_speed']
        print(f'Total time taken: ', total_time / 3600, ' hrs')
        total_energy = total_distance * self.drone.E_cr + len(self.windfarm.turbines) * self.drone.E_ins
        total_hydro = total_energy / 34000
        print(f'Total hydrogen consumed: {total_hydro} g')

    def run_model(self, max_runtime=10):
        def distance_callback(from_index, to_index):
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            distance = self.distance_matrix[from_node][to_node]
            if to_node != 0:
                distance += data['distance_spent_turbine']  # Inspection cost in distance

            return distance

        def add_distance_dimension(routing, manager, data, distance_callback_index):
            distance = "Distance"
            routing.AddDimension(
                distance_callback_index,
                0,  # null slack
                data['vehicle_max_distance'],
                True,  # start cumul to zero
                distance
            )

            distance_dimension = routing.GetDimensionOrDie(distance)
            # Try to minimize the max distance among vehicles.
            # /!\ It doesn't mean the standard deviation is minimized
            # distance_dimension.SetGlobalSpanCostCoefficient(100)

        data = self.create_data_model()

        # Create routing index manager
        manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                               data['num_vehicles'], data['depot'])

        # Create routing model
        routing = pywrapcp.RoutingModel(manager)

        transit_callback_index = routing.RegisterTransitCallback(distance_callback)

        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        add_distance_dimension(routing, manager, data, transit_callback_index)

        # Setting first solution heuristic
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )
        #
        search_parameters.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
        search_parameters.time_limit.FromSeconds(max_runtime)
        # Solve problem
        solution = routing.SolveWithParameters(search_parameters)

        if solution:
            self.print_solution(data, manager, routing, solution)
        else:
            print('No solution found!')


if __name__ == "__main__":
    prop = ShelfPropeller("T-Motor NS 26x85")
    motor = ShelfMotor("T-Motor Antigravity MN6007II KV160")
    esc = ShelfESC("T-Motor FLAME 60A")

    drone_route = DroneRoute(propeller=prop, motor=motor, esc=esc, tank_mass=1.65)

    hornsea = WindFarm(limit=None)
    vrpsolver = VRPSolver(hornsea, drone_route, n_trips=35)
    vrpsolver.run_model(max_runtime=10)
