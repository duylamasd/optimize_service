import json
import sys
import os
from ortools.algorithms import pywrapknapsack_solver

from flask import jsonify, request
from flask.views import MethodView


class BppSolver(MethodView):
    """
    Bin packing solver with multiple dimensions.
    """

    def post(self):
        data = request.get_json()
        profits = data['profits']
        weights = data['weights']
        capacities = data['capacities']

        solver = pywrapknapsack_solver.KnapsackSolver(
            pywrapknapsack_solver.KnapsackSolver.KNAPSACK_MULTIDIMENSION_BRANCH_AND_BOUND_SOLVER, "bpp_solver")
        solver.Init(profits, weights, capacities)
        computed_value = solver.Solve()
        packed_items = [x for x in range(0, len(weights[0])) if solver.BestSolutionContains(x)]
        packed_weights = [weights[0][i] for i in packed_items]
        total_weight = sum(packed_weights)

        return jsonify({
            'packed_items': packed_items,
            'total_profit': computed_value,
            'total_weight': total_weight
        })