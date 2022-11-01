""" Exceptions and errors for deep learning """
    
class TipNotFoundError(Exception):
    """
    Exception raised when pipette tip is not detected
    """

    def __init__(self, message: str = "No pipette tip detected."):
        self.message = message

    def __str__(self):
        return self.message

    def __repr__(self):
        return self.message
