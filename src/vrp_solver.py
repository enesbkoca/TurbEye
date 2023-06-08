import numpy as np
from src.windfarm import WindFarm
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


class VRPSolver:
    def __init__(self, windfarm):
        self.windfarm = windfarm
        self.coordinates = np.vstack((np.array([[0, 0]]), self.windfarm.coordinates)).astype('float')[0:10]
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

        return dists

    def create_data_model(self):
        data = dict()
        data["distance_matrix"] = self.distance_matrix
        data["num_vehicles"] = 1
        data["depot"] = 0
        return data

    def print_solution(self, data, manager, routing, solution):
        """Prints solution on console."""
        print(f'Objective: {solution.ObjectiveValue()}')
        max_route_distance = 0
        for vehicle_id in range(data['num_vehicles']):
            index = routing.Start(vehicle_id)
            plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
            route_distance = 0
            while not routing.IsEnd(index):
                plan_output += ' {} -> '.format(manager.IndexToNode(index))
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                route_distance += routing.GetArcCostForVehicle(
                    previous_index, index, vehicle_id)
            plan_output += '{}\n'.format(manager.IndexToNode(index))
            plan_output += 'Distance of the route: {}m\n'.format(route_distance)
            print(plan_output)
            max_route_distance = max(route_distance, max_route_distance)
        print('Maximum of the route distances: {}m'.format(max_route_distance))

    def run_model(self):
        data = self.create_data_model()

        # Create routing index manager
        manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                               data['num_vehicles'], data['depot'])

        # Create routing model
        routing = pywrapcp.RoutingModel(manager)

        def distance_callback(from_index, to_index):
            # Distance between two nodes
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return data['distance_matrix'][from_node][to_node]

        transit_callback_index = routing.RegisterTransitCallback(distance_callback)

        # Define cost of each arc
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        # Add distance constraint.
        dimension_name = 'Distance'
        routing.AddDimension(
            transit_callback_index,
            0,  # no slack
            49000,  # max travel distance
            True,  # start cumul to zero
            dimension_name
        )

        distance_dimension = routing.GetDimensionOrDie(dimension_name)
        distance_dimension.SetGlobalSpanCostCoefficient(100)

        # Setting first solution heuristic
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )

        # Solve problem
        solution = routing.SolveWithParameters(search_parameters)

        if solution:
            self.print_solution(data, manager, routing, solution)
        else:
            print('No solution found!')


if __name__ == "__main__":
    hornsea = WindFarm()
    vrpsolver = VRPSolver(hornsea)
    vrpsolver.run_model()
