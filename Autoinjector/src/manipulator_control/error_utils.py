class CalibrationError(Exception):
    '''
    Exception raised for errors with calibration process

    Attributes:
        msg (str): Explanation of error
    '''
    def __init__(self, msg="Calibration error occured"):
        self.msg = msg
        super().__init__(self.msg)

class CalibrationDataError(Exception):
    '''
    Exception raised for errors with calibration data

    Attributes:
        msg (str): Explanation of error
    '''
    def __init__(self, msg="Error occured with calibration data"):
        self.msg = msg
        super().__init__(self.msg)

class CalibrationDNEError(Exception):
    '''
    Exception raised for errors when calibration doesn't exist

    Attributes:
        msg (str): Explanation of error
    '''
    def __init__(self, msg="Calibration doesn't exist"):
        self.msg = msg
        super().__init__(self.msg)

class CalibrationFileError(Exception):
    '''
    Exception raised for errors when calibration file doesn't exist

    Attributes:
        msg (str): Explanation of error
    '''
    def __init__(self, msg="Calibration file doesn't exist"):
        self.msg = msg
        super().__init__(self.msg)

class AngleFileError(Exception):
    '''
    Exception raised for errors when Angle file doesn't exist

    Attributes:
        msg (str): Explanation of error
    '''
    def __init__(self, msg="Angle file doesn't exist"):
        self.msg = msg
        super().__init__(self.msg)

class TrajectoryError(Exception):
    '''
    Exception raised for errors with trajectory

    Attributes:
        msg (str): Explanation of error
    '''
    def __init__(self, msg="Trajectory error"):
        self.msg = msg
        super().__init__(self.msg)