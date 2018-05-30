from flask import jsonify

class ErrorHandler:
    def bad_request(self, message):
        """
        Error 400 handler.
        """
        response = jsonify({
            'status': 400,
            'error': 'bad request',
            'message': message
        })
        response.status_code = 400
        return response

    def method_not_allowed(self, message):
        """
        Error 405 handler
        """
        response = jsonify({
            'status': 405,
            'error': 'method not allowed',
            'message': message
        })
        response.status_code = 405
        return response

    def not_found(self, message):
        """
        Error 404 handler
        """
        response = jsonify({
            'status': 404,
            'error': 'not found',
            'message': message
        })
        response.status_code = 404
        return response

class ExceptionHandler(Exception):
    """
    Exception handler
    """

    def __init__(self, message, status_code = None, payload = None):
        Exception.__init__(self)
        self.message = message
        if status_code is not None:
            self.status_code = status_code
        self.payload = payload

    def convert2Dict(self):
        """
        Convert error payload to dictionary
        """
        result = dict(self.payload or ())
        result['message'] = self.message
        return result