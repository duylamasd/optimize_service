from rectpack import newPacker
from flask import jsonify, request
from flask.views import MethodView

import json
import sys
import os

class Bpp2dSolver(MethodView):
    """
    2D Bin packing solver
    """
    def post(self):
        data = request.get_json()
        rectangles = data['rectangles']
        bins = data['bins']

        # Packing protocol
        packer = newPacker()

        # Add the rectangles to packing queue
        for i, r in enumerate(rectangles):
            packer.add_rect(r['width'], r['height'], i)

        # Add the bins where the rectangles will be placed
        for b in bins:
            packer.add_bin(b['width'], b['height'])

        # Start packing
        packer.pack()

        # Get the packing result.
        response = { 'packing': [] }
        all_rects = packer.rect_list()
        for rect in all_rects:
            b, x, y, w, h, rid = rect
            response['packing'].append({ 'bin': b, 'rect': rid, 'x': x, 'y': y, 'w': w, 'h': h })

        return jsonify(response)
