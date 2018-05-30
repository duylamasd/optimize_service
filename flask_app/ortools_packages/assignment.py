from ortools.linear_solver import pywraplp

import json
import sys
import os

class AssignmentProtocol(object):
    """
    An assignment protocol has these following parameters
        - costs: a 2d array to estimate the cost of task when assign order x to trip y
        - order_weights: array of weights of orders
        - order_cbms: array of cbms of orders
        - max_weights, max_cbms: trip capacities

    Methods:
        - Assign: find the assignment
    """
    def __init__(self, costs, order_weights, order_cbms, max_weights, max_cbms):
        self.costs = costs
        self.order_weights = order_weights
        self.order_cbms = order_cbms
        self.max_weights = max_weights
        self.max_cbms = max_cbms
        # Instantiate a mixed-integer solver.
        self.solver = pywraplp.Solver('SolveAssignmentProblemMIP',
                            pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)

    def Assign(self):
        """
        Return assignment as json string. The json string format is:
            {
                'assignment': [
                    {
                        'order': order_no,
                        'trip': trip_no
                    }
                ]
            }
        """
        num_orders = len(self.costs)
        num_trips = len(self.costs[0])
        x = {}

        for i in range(num_orders):
            for j in range(num_trips):
                x[i, j] = self.solver.BoolVar('x[%i,%i]' % (i, j))
        
        # Objective
        self.solver.Minimize(self.solver.Sum([self.costs[i][j] * x[i, j] for i in range(num_orders)
                                                        for j in range(num_trips)]))
        
        # Constraints
        # Total orders assigned doesn't exceed max weight of trip
        for i in range(num_trips):
            self.solver.Add(self.solver.Sum([x[j, i] * self.order_weights[j] for j in range(num_orders)]) <= self.max_weights[i])

        # Total CBM assigned doesn't exceed max CBM of trip
        for i in range(num_trips):
            self.solver.Add(self.solver.Sum([x[j, i] * self.order_cbms[j] for j in range(num_orders)]) <= self.max_cbms[i])

        # Each order is assigned to at most 1 trip
        for i in range(num_orders):
            self.solver.Add(self.solver.Sum([x[i, j] for j in range(num_trips)]) <= 1)

        # All orders must be assigned
        self.solver.Add(self.solver.Sum([x[i, j] for i in range(num_orders) for j in range(num_trips)]) == num_orders)

        solve = self.solver.Solve()
        json_data = {
            'assignment': []
        }
        for i in range(num_orders):
            for j in range(num_trips):
                if x[i, j].solution_value() > 0:
                    json_data['assignment'].append({ 'order': i, 'trip': j })
        
        # Return result
        return json_data
    

def main():
    file_path = sys.argv[1]
    input_str = open(file_path, 'r').read()
    input_data = json.loads(input_str)
    if os.path.isfile(file_path):
        os.remove(file_path)

    costs = input_data['costs']
    order_weights = input_data['order_weights']
    order_cbms = input_data['order_cbms']
    max_weights = input_data['max_weights']
    max_cbms = input_data['max_cbms']

    # Instantiate a mixed-integer solver.
    solver = pywraplp.Solver('SolveAssignmentProblemMIP',
                            pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)
    
    num_orders = len(costs)
    num_trips = len(costs[0])
    x = {}

    for i in range(num_orders):
        for j in range(num_trips):
            x[i, j] = solver.BoolVar('x[%i,%i]' % (i, j))
    
    # Objective
    solver.Minimize(solver.Sum([costs[i][j] * x[i, j] for i in range(num_orders)
                                                    for j in range(num_trips)]))
    
    # Constraints
    # Total orders assigned doesn't exceed max weight of trip
    for i in range(num_trips):
        solver.Add(solver.Sum([x[j, i] * order_weights[j] for j in range(num_orders)]) <= max_weights[i])

    # Total CBM assigned doesn't exceed max CBM of trip
    for i in range(num_trips):
        solver.Add(solver.Sum([x[j, i] * order_cbms[j] for j in range(num_orders)]) <= max_cbms[i])

    # Each order is assigned to at most 1 trip
    for i in range(num_orders):
        solver.Add(solver.Sum([x[i, j] for j in range(num_trips)]) <= 1)

    # Each trip is assigned to at least 1 order
    for i in range(num_trips):
        solver.Add(solver.Sum([x[j, i] for j in range(num_orders)]) >= 1)

    solve = solver.Solve()
    json_data = {
        'assignment': []
    }
    for i in range(num_orders):
        for j in range(num_trips):
            if x[i, j].solution_value() > 0:
                json_data['assignment'].append({ 'order': i, 'trip': j })
    
    # Print result
    print json.dumps(json_data)

if __name__ == '__main__':
    main()