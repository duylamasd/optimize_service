from __future__ import print_function
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import solver_parameters_pb2
import json
import sys
import os

from flask import jsonify, request
from flask.views import MethodView

sys.path.append('..')
from errors import ExceptionHandler


class MipSolver(MethodView):
    """
    Mix Integer Programming solver for assignment problem 
    """

    def post(self):
        data = request.get_json()
        arr = data['array']
        demand = data['demand']

        response_data = { 'data': [] }
        for obj in arr:
            c = map(int, obj['list_weights'])
            costs = map(int, obj['costs'])
            num_trucks = len(c)

            # Instantiate a CP solver.
            parameters = pywrapcp.Solver.DefaultSolverParameters()
            solver = pywrapcp.Solver("mip_solver", parameters)

            # array number for each type of truck
            x = [solver.IntVar(0, 1000, "x%i" % i) for i in range(num_trucks)]

            formula = x[0] * c[0]
            for i in range(1, num_trucks):
                formula = formula + (x[i] * c[i])
            max_weight = max(c)
            solver.Add(formula >= demand)
            solver.Add(formula < (demand + max_weight))

            cost_formula = x[0] * costs[0]
            for i in range(1, num_trucks):
                cost_formula = cost_formula + (x[i] * costs[i])
            obj_expr = solver.IntVar(0, 100000000, 'obj_expr')
            solver.Add(obj_expr == cost_formula)
            objective = solver.Minimize(obj_expr, 1)
            decision_builder = solver.Phase(x, solver.CHOOSE_LOWEST_MIN, solver.ASSIGN_MIN_VALUE)

            collector = solver.LastSolutionCollector()
            for i in x: collector.Add(i)
            collector.AddObjective(obj_expr)
            solver.Solve(decision_builder, [objective, collector])
            if collector.SolutionCount() > 0:
                response_data['data'].append(print_result(collector, x))
            else:
                raise ExceptionHandler(message = "No solution found.", status_code = 400)

        return jsonify(response_data)

def print_result(collector, x):
    list_result = []
    idx = collector.SolutionCount() - 1

    total_cost = collector.ObjectiveValue(idx)
    for value in x: list_result.append(collector.Value(idx, value))

    json_obj = {
        'total_cost': total_cost,
        'list_result': list_result
    }
    return json_obj
