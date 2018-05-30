from flask import jsonify, request
from flask.views import MethodView
from ortools.graph import pywrapgraph

import sys
import os

sys.path.append('..')
from errors import ExceptionHandler


class MinCostFlowsSolver(MethodView):
    """
    Min cost flows solver
    """

    def post(self):
        data = request.get_json()
        starts = data['starts']
        ends = data['ends']
        costs = data['costs']
        supplies = data['supplies']
        source = data['source']
        sink = data['sink']

        min_cost_flow = pywrapgraph.SimpleMinCostFlow()

        for i in range(len(starts)):
            min_cost_flow.AddArcWithCapacityAndUnitCost(
                starts[i], ends[i], 1, costs[i])

        for i in range(len(supplies)):
            min_cost_flow.SetNodeSupply(i, supplies[i])

        if min_cost_flow.Solve() == min_cost_flow.OPTIMAL:
            response = {
                'total': min_cost_flow.OptimalCost(),
                'arcs': []
            }

            for arc in range(min_cost_flow.NumArcs()):
                if min_cost_flow.Flow(arc) > 0:
                    response['arcs'].append({
                        'arc': arc,
                        'tail': min_cost_flow.Tail(arc),
                        'head': min_cost_flow.Head(arc),
                        'cost': min_cost_flow.UnitCost(arc)
                    })

            return jsonify(response)
        else:
            raise ExceptionHandler(
                message="No solution found", status_code=400)
