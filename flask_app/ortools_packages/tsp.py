from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

import itertools
import json
import numpy as np
import sys
import os

_INFINITE = 10000000

class DistanceMatrix(object):
    """
    Create distance matrix from other matrix.
    """
    def __init__(self, matrix):
        self.matrix = matrix

    def Distance(self, from_node, to_node):
        """
        Get the distance from matrix.
        """
        return int(self.matrix[from_node][to_node])

class DistanceMatrixFromListLocation(object):
    """
    Create distance matrix from list locations.
    """
    def __init__(self, locations):
        matrix_len = len(locations)
        self.matrix = np.empty([matrix_len, matrix_len])
        for i, j in itertools.product(range(matrix_len), range(matrix_len)):
            self.matrix[i, j] = self.calculate_distance(locations[i], locations[j])
        
    @staticmethod
    def calculate_distance(a, b):
        """
        Calculate distance between location a and b.
        """
        lat1, lon1, lat2, lon2 = map(np.radians, [a[0], a[1], b[0], b[1]])
        # haversine formula
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        s = (np.sin(dlat / 2) ** 2 + np.cos(lat1) *
                np.cos(lat2) * np.sin(dlon / 2) ** 2)
        c = 2 * np.arcsin(np.sqrt(s))

        # 6367 km is the radius of the Earth
        km = 6367 * c
        return km

    def Distance(self, from_node, to_node):
        """
        Get the distance from matrix.
        """
        return int(self.matrix[from_node][to_node])

    def get_matrix(self):
        """
        Return the distance matrix.
        """
        return self.matrix.tolist()

class TSPSolver(object):
    """
    A solver to solve TSP.
    """
    def __init__(self, matrix):
        self.matrix = matrix

    def SolveTSP(self):
        """
        Solve TSP method.
        """
        tsp_size = len(self.matrix)
        num_routes = 1
        # Create routing model
        if tsp_size > 2:
            first_routing = pywrapcp.RoutingModel(tsp_size, num_routes, [0], [tsp_size - 1])
            second_routing = pywrapcp.RoutingModel(tsp_size, num_routes, [0], [tsp_size - 2])
            search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()

            # Create the distance callback, which takes two arguments (the from and to node indices)
            # and returns the distance between these nodes.
            dist_between_nodes = DistanceMatrix(self.matrix)
            dist_callback = dist_between_nodes.Distance
            first_routing.SetArcCostEvaluatorOfAllVehicles(dist_callback)
            second_routing.SetArcCostEvaluatorOfAllVehicles(dist_callback)
            # Solve, returns a solution if any.
            first_assignment = first_routing.SolveWithParameters(search_parameters)
            second_assignment = second_routing.SolveWithParameters(search_parameters)

            # Create result data of 2 routing models
            first_result_data = {
                'total': _INFINITE,
                'route_detail': []
            }
            second_result_data = {
                'total': _INFINITE,
                'route_detail': []
            }

            if first_assignment:
                first_result_data = {
                    'total': first_assignment.ObjectiveValue(),
                    'route_detail': []
                }
                # Inspect solution.
                # Only one route here; otherwise iterate from 0 to routing.vehicles() - 1
                route_number = 0
                index = first_routing.Start(route_number) # Index of the variable for the starting node.
                while not first_routing.IsEnd(index):
                    # Convert variable indices to node indices in the displayed route.
                    # Result is the array of indices of nodes.
                    first_result_data['route_detail'].append(first_routing.IndexToNode(index))
                    index = first_assignment.Value(first_routing.NextVar(index))
                first_result_data['route_detail'].append(first_routing.IndexToNode(index))

            if second_assignment:
                second_result_data = {
                    'total': second_assignment.ObjectiveValue(),
                    'route_detail': []
                }
                # Inspect solution.
                # Only one route here; otherwise iterate from 0 to routing.vehicles() - 1
                route_number = 0
                index = second_routing.Start(route_number) # Index of the variable for the starting node.
                while not second_routing.IsEnd(index):
                    # Convert variable indices to node indices in the displayed route.
                    # Result is the array of indices of nodes.
                    second_result_data['route_detail'].append(second_routing.IndexToNode(index))
                    index = second_assignment.Value(second_routing.NextVar(index))
                second_result_data['route_detail'].append(second_routing.IndexToNode(index))
            
            if not (first_assignment or second_assignment):
                raise Exception('No solution found')
            else:
                # Select the one has smallest total distance of route, then dump it to json string.
                json_result = first_result_data if (first_result_data['total'] < second_result_data['total']) else second_result_data
                return json_result
        else:
            raise Exception('Specify an instance greater than 2.')

def main():
    file_path = sys.argv[1]
    input_str = open(file_path, 'r').read()
    input_data = json.loads(input_str)
    if os.path.isfile(file_path):
        os.remove(file_path)

    matrix = input_data['matrix']
    tsp_size = len(matrix)
    num_routes = 1

    # Create routing model
    if tsp_size > 1:
        routing = pywrapcp.RoutingModel(tsp_size, num_routes, [0], [tsp_size - 1])
        search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()

        # Create the distance callback, which takes two arguments (the from and to node indices)
        # and returns the distance between these nodes.
        dist_between_nodes = DistanceMatrixFromListLocation(matrix)
        dist_callback = dist_between_nodes.Distance
        routing.SetArcCostEvaluatorOfAllVehicles(dist_callback)
        # Solve, returns a solution if any.
        assignment = routing.SolveWithParameters(search_parameters)

        # Create result data of 2 routing models
        result_data = {
            'total': _INFINITE,
            'route_detail': []
        }

        if assignment:
            result_data = {
                'total': assignment.ObjectiveValue(),
                'route_detail': []
            }
            # Inspect solution.
            # Only one route here; otherwise iterate from 0 to routing.vehicles() - 1
            route_number = 0
            index = routing.Start(route_number) # Index of the variable for the starting node.
            while not routing.IsEnd(index):
                # Convert variable indices to node indices in the displayed route.
                # Result is the array of indices of nodes.
                result_data['route_detail'].append(routing.IndexToNode(index))
                index = assignment.Value(routing.NextVar(index))
            result_data['route_detail'].append(routing.IndexToNode(index))

            # dump the result to json string.
            json_str = json.dumps(result_data)
            print json_str
        else:
            raise Exception('No solution found')

    else:
        raise Exception('Specify an instance greater than 2.')

if __name__ == '__main__':
    main()