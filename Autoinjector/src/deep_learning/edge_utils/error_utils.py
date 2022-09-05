""" Exceptions and errors for edge classification """


class EdgeNotFoundError(Exception):
    """
    Exception raised when edge is not found in image
    """

    def __init__(self, message: str = "No edge detected."):
        self.message = message

    def __str__(self):
        return self.message

    def __repr__(self):
        return self.message
