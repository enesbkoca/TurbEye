import numpy as np
from src.windfarm import WindFarm

from functools import partial

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


class VRPSolver:
    def __init__(self, windfarm, max_trips=50):
        self.windfarm = windfarm
        self.max_trips = max_trips
        self.n_turbines = len(self.windfarm.turbines)
        self.coordinates = np.vstack((np.tile(np.array([[0, 0]]), (self.max_trips, 1)), self.windfarm.coordinates)).astype('float')
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
        _capacity = 1000
        data["distance_matrix"] = self.distance_matrix
        data["num_vehicles"] = 1
        data["depot"] = 0
        data["distance_spent_turbine"] = 40_820  # m
        data["vehicle_max_distance"] = 27 * 60 * 60 * 3
        data["demands"] = np.array([0] + [_capacity] * (self.max_trips - 1) + [1] * self.n_turbines)
        data["n_locations"] = len(data["demands"])
        data["vehicle_capacity"] = _capacity

        self.data = data

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
        def create_distance_evaluator(data):
            _distances = {}

            for from_node in range(data['n_locations']):
                _distances[from_node] = {}
                for to_node in range(data["n_locations"]):
                    if from_node == to_node:
                        _distances[from_node][to_node] = 0
                    elif from_node in range(self.max_trips) and to_node in range(self.max_trips):
                        _distances[from_node][to_node] = data["vehicle_max_distance"]
                    else:
                        distance = self.distance_matrix[from_node][to_node]
                        if to_node >= self.max_trips:
                            distance += data['distance_spent_turbine']  # Inspection cost in distance

                        _distances[from_node][to_node] = distance

            def distance_evaluator(manager, from_node, to_node):
                return _distances[manager.IndexToNode(from_node)][manager.IndexToNode(to_node)]

            return distance_evaluator

        def create_demand_evaluator(data):
            _demands = data["demands"]

            def demand_evaluator(manager, from_node):
                return _demands[manager.IndexToNode(from_node)]
            return demand_evaluator

        def add_capacity_constraints(routing, manager, data, demand_evaluator_index):
            vehicle_capacity = data['vehicle_capacity']
            capacity = 'Capacity'
            routing.AddDimension(
                demand_evaluator_index,
                vehicle_capacity,
                vehicle_capacity,
                True,
                capacity
            )

            capacity_dimension = routing.GetDimensionOrDie(capacity)

            for node in range(self.max_trips):
                node_index = manager.IndexToNode(node)
                routing.AddDisjunction([node_index], 0)

            for node in range(self.max_trips, len([data['demands']])):
                node_index = manager.NodeToIndex(node)
                capacity_dimension.SlackVar(node_index).SetValue(0)
                routing.AddDisjunction([node_index], 100-000)


        def add_distance_dimension(routing, manager, data, distance_evaluator_index):
            distance = "Distance"
            routing.AddDimension(
                distance_evaluator_index,
                0,  # null slack
                data['vehicle_max_distance'],
                True,  # start cumul to zero
                distance
            )

            distance_dimension = routing.GetDimensionOrDie(distance)
            # Try to minimize the max distance among vehicles.
            # /!\ It doesn't mean the standard deviation is minimized
            distance_dimension.SetGlobalSpanCostCoefficient(100)

        data = self.create_data_model()

        # Create routing index manager
        manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                               data['num_vehicles'], data['depot'])

        # Create routing model
        routing = pywrapcp.RoutingModel(manager)

        distance_evaluator_index = routing.RegisterTransitCallback(
            partial(create_distance_evaluator(data), manager)
        )
        routing.SetArcCostEvaluatorOfAllVehicles(distance_evaluator_index)

        add_distance_dimension(routing, manager, data, distance_evaluator_index)

        demand_evaluator_index = routing.RegisterUnaryTransitCallback(
            partial(create_demand_evaluator(data), manager))
        add_capacity_constraints(routing, manager, data, demand_evaluator_index)

        # Setting first solution heuristic
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )

        search_parameters.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
        search_parameters.time_limit.FromSeconds(100)
        # Solve problem
        solution = routing.SolveWithParameters(search_parameters)

        if solution:
            self.print_solution(data, manager, routing, solution)
        else:
            print('No solution found!')


if __name__ == "__main__":
    hornsea = WindFarm(limit=8)
    vrpsolver = VRPSolver(hornsea, max_trips=5)
    # print(vrpsolver.distance(0, 1) + vrpsolver.distance(1, 2) + vrpsolver.distance(2, 3) + vrpsolver.distance(3, 4) + vrpsolver.distance(4, 5) + vrpsolver.distance(5, 0))
    vrpsolver.run_model()
