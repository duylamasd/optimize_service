import json
import sys
import numpy as np
import itertools
import os

import tsp
import assignment

_INFINITE = 10000000

class DistanceMatrix(object):
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

def main():
    file_path = sys.argv[1]
    input_str = open(file_path, 'r').read()
    input_data = json.loads(input_str)
    if os.path.isfile(file_path):
        os.remove(file_path)

    trips = input_data['trips']
    orders = input_data['orders']
    num_trips = len(trips)
    num_orders = len(orders)
    
    # Generate costs matrix
    costs = []
    for i in range(num_orders):
        costs.append([0 for j in range(num_trips)])
    for i, order in enumerate(orders):
        locations = [(order['lats'][0], order['lons'][0])]
        for j, trip in enumerate(trips):
            # Generate locations list
            for idx in range(len(trip['lats'])):
                locations.append((trip['lats'][idx], trip['lons'][idx]))
            locations.append((order['lats'][1], order['lons'][1]))

            # Create distance matrix, then calculate the cost (as distance)
            dist_matrix = DistanceMatrix(locations).get_matrix()
            cost_data = tsp.TSPSolver(dist_matrix).SolveTSP()
            costs[i][j] = cost_data['total']
    
    # Generate data for assignment
    order_weights = [order['weight'] for order in orders]
    order_cbms = [order['cbm'] for order in orders]
    max_weights = [trip['max_weight'] for trip in trips]
    max_cbms = [trip['max_cbm'] for trip in trips]

    # Create assignment protocol
    assignment_protocol = assignment.AssignmentProtocol(costs, order_weights, order_cbms, max_weights, max_cbms)
    assignment_result = assignment_protocol.Assign()

    # Print assignment
    print json.dumps(assignment_result)

if __name__ == '__main__':
    main()