from ortools.linear_solver import pywraplp

import json
import sys
import os

def main():
    file_path = sys.argv[1]
    input_str = open(file_path, 'r').read()
    input_data = json.loads(input_str)
    if os.path.isfile(file_path):
        os.remove(file_path)

    costs = input_data['costs']
    num_vendors = len(costs)
    num_orders = len(costs[0])
    x = {}

    solver = pywraplp.Solver('SolveAssignmentProblemMIP',
                            pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)

    for i in range(num_vendors):
            for j in range(num_orders):
                x[i, j] = solver.BoolVar('x[%i,%i]' % (i, j))

    # Objective
    solver.Minimize(solver.Sum([costs[i][j] * x[i, j] for i in range(num_vendors)
                                                        for j in range(num_orders)]))

    # Constraints
    # All orders must be assigned
    solver.Add(solver.Sum([x[i, j] for i in range(num_vendors) for j in range(num_orders)]) == num_orders)

    solve = solver.Solve()
    json_data = {
        'assignment': []
    }
    for i in range(num_vendors):
        for j in range(num_orders):
            if x[i, j].solution_value() > 0:
                json_data['assignment'].append({ 'vendor': i, 'order': j })

    print json.dumps(json_data)

if __name__ == '__main__':
    main()