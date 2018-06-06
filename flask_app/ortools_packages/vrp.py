import json
import numpy as np
import sys
import datetime
import os
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
from collections import namedtuple
from copy import deepcopy

from flask import jsonify, request
from flask.views import MethodView

sys.path.append('..')
from errors import ExceptionHandler


DISTANCE_INF = 1000


class Evaluator:
    """
    Callbacks handler. This contained these methods.
    - Time callback.
    - Demand callback.
    - Distance callback.
    """

    def total_time(self, demands, locations, loadings, unloadings, speed=40):
        def service_time_return(a, b):
            return loadings[a] + unloadings[a]

        def transit_time_return(a, b):
            return (self.distance(locations[a], locations[b]) / speed) * 3600

        def total_time_return(a, b):
            if transit_time_return(a, b) == 0:
                return 0
            return service_time_return(a, b) + transit_time_return(a, b)
        return total_time_return

    # using haversine formula for computing distance
    @staticmethod
    def distance(a, b):
        lat1, lon1, lat2, lon2 = map(np.radians, [a[0], a[1], b[0], b[1]])
        # haversine formula
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        s = (np.sin(dlat / 2) ** 2 + np.cos(lat1)
             * np.cos(lat2) * np.sin(dlon / 2) ** 2)
        c = 2 * np.arcsin(np.sqrt(s))

        # 6367 km is the radius of the Earth
        km = 6367 * c
        return km * 1.4

    def distance_callback(self, locations, matrix):
        def distance_calculate(a, b):
            if matrix[a][b]:
                return self.distance(locations[a], locations[b])
            return DISTANCE_INF
        return distance_calculate

    def demands_calculate(self, demands):
        def get_demand(a, b):
            return demands[a]
        return get_demand


class VrpSolver(MethodView):
    """
    Solver for Vehicle Routing Problem
    """

    def post(self):
        # region Input data
        data = request.get_json()
        allow_drop = data['allow_drop']
        departure_times = data['departure_times']
        vehicle_capacities = data['vehicle_capacities']
        vehicle_costs = data['vehicle_costs']

        # Gererate locations coordinate
        lats = data['lats']
        lons = data['lons']
        num_locations = len(lats)
        locations = [None for lat in lats]
        for idx in range(num_locations):
            locations[idx] = (lats[idx], lons[idx])

        departure_depots = data['departure_depots']
        return_depots = data['return_depots']
        start_times = data['start_times']
        end_times = data['end_times']
        return_times = data['return_times']
        demands = data['demands']
        matrix = data['matrix']
        groups = data['groups']
        velocities = data['velocities']
        horizon = data['horizon']
        loadings = data['loadings']
        unloadings = data['unloadings']
        min_weights = data['min_weights']
        first_vendor_index = data['first_vendor_index']

        num_vehicles = len(vehicle_capacities)
        # endregion

        # region Create evaluators and add constrains.
        evaluator = Evaluator()
        dist_callback = evaluator.distance_callback(locations, matrix)
        demands_callback = evaluator.demands_calculate(demands)
        total_time_callbacks = []
        for i in range(num_vehicles):
            total_time_callbacks.append(deepcopy(evaluator.total_time(
                demands=demands, locations=locations, loadings=loadings, unloadings=unloadings, speed=velocities[i])))

        routing = pywrapcp.RoutingModel(
            num_locations, num_vehicles, departure_depots, return_depots)
        routing.SetArcCostEvaluatorOfAllVehicles(dist_callback)
        for index, cost in enumerate(vehicle_costs):
            routing.SetFixedCostOfVehicle(cost, index)

        search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC)
        search_parameters.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)

        search_parameters.time_limit_ms = 30 * 1000

        routing.AddDimensionWithVehicleCapacity(evaluator=demands_callback,
                                                slack_max=0,
                                                vehicle_capacities=vehicle_capacities,
                                                fix_start_cumul_to_zero=True,
                                                name="capacity")

        routing.AddDimensionWithVehicleTransits(evaluators=total_time_callbacks,
                                                slack_max=horizon,
                                                capacity=horizon,
                                                fix_start_cumul_to_zero=False,
                                                name="time")

        routing.AddDimension(evaluator=dist_callback,
                             slack_max=0,
                             capacity=1000,
                             fix_start_cumul_to_zero=True,
                             name="distance")

        time_dimension = routing.GetDimensionOrDie("time")
        for location in range(num_locations):
            start = int(3600 * start_times[location])
            end = int(3600 * end_times[location])
            time_dimension.CumulVar(location).SetRange(start, end)

        for vehicle in range(num_vehicles):
            start = int(3600 * departure_times[vehicle])
            end = int(3600 * return_times[vehicle])
            time_dimension.CumulVar(routing.Start(vehicle)).SetValue(start)
            time_dimension.CumulVar(routing.End(vehicle)).SetRange(start, end)

        for i in range(num_locations):
            for j in range(num_locations):
                if routing.NextVar(i).Contains(j):
                    if matrix[i][j] == 0:
                        routing.NextVar(i).RemoveValue(j)

        for group in groups:
            routing.AddSoftSameVehicleConstraint(group, 40)

        min_load = 1
        capacity_dimension = routing.GetDimensionOrDie("capacity")

        for vehicle in range(num_vehicles):
            if vehicle < first_vendor_index:
                if allow_drop > 0:
                    capacity_dimension.CumulVar(routing.End(
                        vehicle)).RemoveInterval(0, min_weights[vehicle])
                else:
                    capacity_dimension.CumulVar(routing.End(
                        vehicle)).RemoveInterval(1, min_weights[vehicle])
            else:
                capacity_dimension.CumulVar(routing.End(
                    vehicle)).RemoveInterval(1, min_weights[vehicle])

        assignment = routing.SolveWithParameters(search_parameters)
        if assignment:
            # print "total distance of all routes:", assignment.objectivevalue(), "\n"
            capacity_dimension = routing.GetDimensionOrDie("capacity")
            time_dimension = routing.GetDimensionOrDie("time")
            distance_dimension = routing.GetDimensionOrDie("distance")

            # initialize json data
            json_data = []

            for j in range(num_vehicles):
                routes = []
                index = routing.Start(j)
                while True:
                    node_index = routing.IndexToNode(index)
                    load_var = capacity_dimension.CumulVar(index)
                    time_var = time_dimension.CumulVar(index)
                    distance_var = distance_dimension.CumulVar(index)

                    # extends route to list
                    obj_append = {
                        'location_no': node_index,
                        'location_latitude': lats[node_index],
                        'location_longitude': lons[node_index],
                        'load': assignment.Value(load_var) / 1000.,
                        'distance': assignment.Value(distance_var),
                        'time_open': assignment.Min(time_var),
                        'time_leave': assignment.Max(time_var)
                    }

                    routes.append(obj_append)
                    if routing.IsEnd(index):
                        break
                    else:
                        index = assignment.Value(routing.NextVar(index))

                # print plan_output

                # add vehicle's routes to list data
                vehicle_json_data = {
                    'vehicle_no': j,
                    'departure_time': departure_times[j],
                    'return_time': return_times[j],
                    'capacity': vehicle_capacities[j],
                    'routes': routes
                }
                json_data.append(vehicle_json_data)

            # parse results to json
            json_object = {
                'total': assignment.ObjectiveValue(),
                'result': json_data
            }
            # save result
            return jsonify(json_object)
        else:
            return 'No solution found.'
        # endregion


class DistanceMatrix(MethodView):
    """
    Distance matrix from list of locations.
    """

    @staticmethod
    def distance(a, b):
        """
        Calculate distance between location a and location b.
        """

        lat1, lng1, lat2, lng2 = map(np.radians, [a['lat'], a['lng'], b['lat'], b['lng']])
        dlng = lng2 - lng1
        dlat = lat2 - lat1
        s = (np.sin(dlat / 2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlng / 2)**2)
        c = 2 * np.arcsin(np.sqrt(s))

        km = 6367 * c
        return km * 1.4

    def post(self):
        data = request.get_json()
        locations = data['locations']

        response = { 'matrix': [] }
        for first_loc in locations:
            row = []
            for second_loc in locations:
                row.append(self.distance(first_loc, second_loc))
            response['matrix'].append(row)

        return jsonify(response)