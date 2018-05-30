from __future__ import print_function
from ortools.graph import pywrapgraph
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
    rows = len(costs)
    cols = len(costs[0])

    assignment = pywrapgraph.LinearSumAssignment()
    for worker in range(rows):
        for task in range(cols):
            if costs[worker][task] != 'unknown':
                assignment.AddArcWithCost(worker, task, costs[worker][task])
    
    solve_status = assignment.Solve()
    if solve_status == assignment.OPTIMAL:
        assignment_result = {
            'assignment': []
        }
        for i in range(0, assignment.NumNodes()):
            assignment_result['assignment'].append({
                'worker': i,
                'task': assignment.RightMate(i)
            })
        
        print(json.dumps(assignment_result))

    if solve_status == assignment.INFEASIBLE:
        raise Exception('No assignment is possible.')

    if solve_status == assignment.POSSIBLE_OVERFLOW:
        raise Exception('Some input costs are too large and may cause an integer overflow.')

if __name__ == '__main__':
    main()