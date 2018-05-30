from flask import Flask, jsonify
from errors import ErrorHandler, ExceptionHandler
from ortools_packages.bpp2d import Bpp2dSolver
from ortools_packages.bpp import BppSolver
from ortools_packages.mip import MipSolver
from ortools_packages.vrp import VrpSolver
from ortools_packages.linear import MinCostFlowsSolver

import os

app = Flask(__name__)
basedir = os.path.abspath(os.path.dirname(__file__))
error_handler = ErrorHandler()

#region Error handlers
@app.errorhandler(ExceptionHandler)
def exception_handler(error):
    response = jsonify(error.convert2Dict())
    response.status_code = error.status_code
    return response

@app.errorhandler(400)
def bad_request_error(error):
    return error_handler.bad_request("Bad request")

@app.errorhandler(405)
def method_not_allowed_error(error):
    return error_handler.method_not_allowed("Method not allowed")

@app.errorhandler(404)
def not_found_error(error):
    return error_handler.not_found("Not found")
#endregion

#region APIs
bpp2dView = Bpp2dSolver.as_view('bpp2dView')
app.add_url_rule('/bpp2d', view_func = bpp2dView, methods=['POST'])

bppView = BppSolver.as_view('bppView')
app.add_url_rule('/bpp', view_func = bppView, methods = ['POST'])

mipView = MipSolver.as_view('mipView')
app.add_url_rule('/mip', view_func = mipView, methods = ['POST'])

vrpView = VrpSolver.as_view('vrpView')
app.add_url_rule('/vrp', view_func = vrpView, methods = ['POST'])

linearView = MinCostFlowsSolver.as_view('linearView')
app.add_url_rule('/min_cost', view_func = linearView, methods = ['POST'])
#endregion

if __name__ == '__main__':
    app.run(debug = True)