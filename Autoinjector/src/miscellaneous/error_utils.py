""" Functions/classes for raising handling errors in the GUI """

class InjectionParameterError(Exception):
    '''
    Exception raised for errors with injectino parameters

    Attributes:
        msg (str): Explanation of error
    '''
    def __init__(self, msg="Injection parameter error occured"):
        self.msg = msg
        super().__init__(self.msg)