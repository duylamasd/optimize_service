from __future__ import print_function
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import solver_parameters_pb2
import json
import sys
import os

def print_result(collector, x):
    list_result = []
    idx = collector.SolutionCount() - 1

    total_cost = collector.ObjectiveValue(idx)
    for value in x: list_result.append(collector.Value(idx, value))

    json_obj = {
        'total_cost': total_cost,
        'list_result': list_result
    }
    json_str = json.dumps(json_obj, indent = 4)
    print (json_str)

def main():
    file_path = sys.argv[1]
    input_arr = open(file_path, 'r').read()
    input_arr = input_arr.split(' ')
    if os.path.isfile(file_path):
        os.remove(file_path)

    # Instantiate a CP solver.
    parameters = pywrapcp.Solver.DefaultSolverParameters()
    solver = pywrapcp.Solver("first_CP", parameters)

    unit_prices = input_arr[0]
    unit_prices = map(int, unit_prices.split(','))

    tons = input_arr[1]
    tons = map(int, tons.split(','))

    demand = int(input_arr[2])

    num_trucks = len(unit_prices)

    # variables (number of each type of truck)
    x = [solver.IntVar(0, 1000, "x%i" % i) for i in range(num_trucks)]

    formula = x[0] * tons[0]
    for i in range(1, num_trucks):
        formula = formula + (x[i] * tons[i])

    min_weight = min(tons)

    solver.Add(formula > (demand - min_weight))
    solver.Add(formula <= demand)

    cost_formula = x[0] * unit_prices[0] * tons[0]
    for i in range(1, num_trucks):
        cost_formula = cost_formula + (x[i] * unit_prices[i] * tons[i])
    obj_expr = solver.IntVar(0, 100000000, 'obj_expr')
    solver.Add(obj_expr == cost_formula)
    objective = solver.Minimize(obj_expr, 1)
    decision_builder = solver.Phase(x, solver.CHOOSE_MIN_SIZE, solver.ASSIGN_CENTER_VALUE)

    collector = solver.LastSolutionCollector()
    for i in x: collector.Add(i)
    collector.AddObjective(obj_expr)
    solver.Solve(decision_builder, [objective, collector])
    if collector.SolutionCount() > 0:
        print_result(collector, x)

if __name__ == '__main__':
    main()
